"""
CubeSat CMG Commander — GUI + PD/VSCMG Controller
- Auto mode: PD + VSCMG allocation with live gain tuning
- Manual mode: direct gimbal angle + wheel RPM control
- cmg_testbed_commander.py UI style 기반
"""

import threading
import math
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import Imu, JointState
import numpy as np


# ============================================================
# Quaternion utilities (scalar-first: [w, x, y, z])
# ============================================================

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_error(q, q_d):
    q_e = quat_mult(quat_conj(q_d), q)
    if q_e[0] < 0:
        q_e = -q_e
    return q_e


def euler_to_quat(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


def quat_to_euler(q):
    w, x, y, z = q
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w*y - z*x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])


# ============================================================
# VSCMG parameters (MATLAB CMG_sim.m)
# ============================================================

N_CMG = 2
I_DB = np.array([3.65e-5, 3.65e-5])
GIMBAL_AXIS = np.array([0.0, 1.0, 0.0])
SPIN_AXIS_0 = np.array([1.0, 0.0, 0.0])
VARPI_NOM = np.array([1.0, -1.0]) * 10000.0 * (2.0 * np.pi / 60.0)
VARPI_MAX = 20000.0 * (2.0 * np.pi / 60.0)
GAMMA_DOT_MAX = 1.0
GIMBAL_KP = 5.0

SRI_LAMBDA0 = 0.01
SRI_LAMBDA_MAX = 0.5
SRI_M_THRESHOLD = 0.01
W_RW = 0.15
K_NULL_VARPI = 0.01

KP_DEFAULT = np.array([0.8, 0.8, 15.0])
KD_DEFAULT = np.array([5.5, 5.5, 0.5])
SPINUP_DURATION = 5.0


# ============================================================
# VSCMG functions
# ============================================================

def get_spin_axis(gamma_i):
    a_cross_s0 = np.cross(GIMBAL_AXIS, SPIN_AXIS_0)
    return np.cos(gamma_i) * SPIN_AXIS_0 + np.sin(gamma_i) * a_cross_s0


def get_torque_axis(gamma_i):
    return np.cross(GIMBAL_AXIS, get_spin_axis(gamma_i))


def get_jacobian(gamma, varpi):
    A = np.zeros((3, N_CMG))
    B = np.zeros((3, N_CMG))
    for i in range(N_CMG):
        s_i = get_spin_axis(gamma[i])
        t_i = get_torque_axis(gamma[i])
        A[:, i] = I_DB[i] * varpi[i] * t_i
        B[:, i] = I_DB[i] * s_i
    return A, B


def allocate_vscmg(tau_cmd, gamma, varpi):
    A, B_rw = get_jacobian(gamma, varpi)
    W_cmg = A @ A.T
    m_sigma = np.sqrt(max(np.linalg.det(W_cmg), 0.0))

    if m_sigma < SRI_M_THRESHOLD:
        lam = SRI_LAMBDA0 + (1.0 - m_sigma / SRI_M_THRESHOLD) ** 2 * SRI_LAMBDA_MAX
    else:
        lam = SRI_LAMBDA0

    w2_inv = np.diag(np.concatenate([
        np.ones(N_CMG), np.ones(N_CMG) / (W_RW ** 2),
    ]))
    D = np.hstack([A, B_rw])
    DW = D @ w2_inv
    DWDt = DW @ D.T + lam ** 2 * np.eye(3)
    D_sri = w2_inv @ D.T @ np.linalg.solve(DWDt, np.eye(3))
    u = -D_sri @ tau_cmd

    N_null = np.eye(2 * N_CMG) - D_sri @ D
    u_null_raw = np.concatenate([
        np.zeros(N_CMG), K_NULL_VARPI * (VARPI_NOM - varpi),
    ])
    u += N_null @ u_null_raw

    gamma_dot = np.clip(u[:N_CMG], -GAMMA_DOT_MAX, GAMMA_DOT_MAX)
    varpi_dot = np.clip(u[N_CMG:], -500.0, 500.0)
    return gamma_dot, varpi_dot, m_sigma


# ============================================================
# ROS 2 Node
# ============================================================

class CubeSatCMGCommander(Node):
    def __init__(self):
        super().__init__('cubesat_cmg_commander')

        # Sensor state
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega = np.zeros(3)
        self.gamma = np.zeros(N_CMG)
        self.varpi = np.zeros(N_CMG)
        self.imu_received = False
        self.joint_received = False

        # Control state
        self.mode = 'manual'
        self.att_err_deg = 0.0
        self.m_sigma = 0.0
        self.auto_start_time = None
        self.auto_phase = 'idle'

        # Gains (updated from GUI)
        self.kp = KP_DEFAULT.copy()
        self.kd = KD_DEFAULT.copy()

        # Target (updated from GUI)
        self.target_euler_deg = np.array([15.0, -10.0, 25.0])

        # Manual commands (from GUI)
        self.cmd_gimbal_deg = [0.0, 0.0]
        self.cmd_rpm = [0.0, 0.0]

        # Subscribers
        self.create_subscription(Imu, '/cubesat_cmg/imu', self.imu_cb, 10)
        self.create_subscription(
            JointState, '/cubesat_cmg/joint_states', self.joint_cb, 10)

        # Publishers
        self.pub_gimbal = [
            self.create_publisher(
                Float64, '/cubesat_cmg/cmg1_gimbal/cmd_vel', 10),
            self.create_publisher(
                Float64, '/cubesat_cmg/cmg2_gimbal/cmd_vel', 10),
        ]
        self.pub_wheel = [
            self.create_publisher(
                Float64, '/cubesat_cmg/cmg1_wheel/cmd_vel', 10),
            self.create_publisher(
                Float64, '/cubesat_cmg/cmg2_wheel/cmd_vel', 10),
        ]
        self.pub_log = self.create_publisher(
            Float64MultiArray, '/cubesat_cmg/attitude_log', 10)

        self.dt = 0.01
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('CubeSat CMG Commander started')

    def imu_cb(self, msg):
        o = msg.orientation
        self.q = np.array([o.w, o.x, o.y, o.z])
        norm_q = np.linalg.norm(self.q)
        if norm_q > 0:
            self.q /= norm_q
        av = msg.angular_velocity
        self.omega = np.array([av.x, av.y, av.z])
        self.imu_received = True

    def joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'cmg1_gimbal_joint' and len(msg.position) > i:
                self.gamma[0] = msg.position[i]
            elif name == 'cmg2_gimbal_joint' and len(msg.position) > i:
                self.gamma[1] = msg.position[i]
            if name == 'cmg1_wheel_joint' and len(msg.velocity) > i:
                self.varpi[0] = msg.velocity[i]
            elif name == 'cmg2_wheel_joint' and len(msg.velocity) > i:
                self.varpi[1] = msg.velocity[i]
        self.joint_received = True

    def control_loop(self):
        # Manual mode: publish commands unconditionally (like cmg_testbed)
        if self.mode != 'auto':
            self._manual_control()
            self.auto_start_time = None
            self.auto_phase = 'idle'
            # Update attitude error display if sensor data available
            if self.imu_received:
                target_rad = np.radians(self.target_euler_deg)
                q_d = euler_to_quat(target_rad[0], target_rad[1], target_rad[2])
                q_e = quat_error(self.q, q_d)
                self.att_err_deg = 2.0 * np.arccos(
                    np.clip(abs(q_e[0]), 0.0, 1.0)) * 180.0 / np.pi
            self._publish_telemetry()
            return

        # Auto mode: requires sensor feedback for PD control
        if not self.imu_received or not self.joint_received:
            return

        # Compute attitude error
        target_rad = np.radians(self.target_euler_deg)
        q_d = euler_to_quat(target_rad[0], target_rad[1], target_rad[2])
        q_e = quat_error(self.q, q_d)
        self.att_err_deg = 2.0 * np.arccos(
            np.clip(abs(q_e[0]), 0.0, 1.0)) * 180.0 / np.pi

        self._auto_control()
        self._publish_telemetry()

    def _publish_telemetry(self):
        euler_deg = np.degrees(quat_to_euler(self.q))
        log_msg = Float64MultiArray()
        log_msg.data = [
            float(self.q[0]), float(self.q[1]),
            float(self.q[2]), float(self.q[3]),
            float(self.omega[0]), float(self.omega[1]), float(self.omega[2]),
            float(self.gamma[0]), float(self.gamma[1]),
            float(self.varpi[0]), float(self.varpi[1]),
            float(self.att_err_deg), float(self.m_sigma),
            float(euler_deg[0]), float(euler_deg[1]), float(euler_deg[2]),
        ]
        self.pub_log.publish(log_msg)

    def _auto_control(self):
        if self.auto_start_time is None:
            self.auto_start_time = self.get_clock().now()
            self.auto_phase = 'spinup'

        elapsed = (
            self.get_clock().now() - self.auto_start_time
        ).nanoseconds / 1e9

        # Phase 1: spin-up
        if elapsed < SPINUP_DURATION:
            self.auto_phase = 'spinup'
            ramp = elapsed / SPINUP_DURATION
            for i in range(N_CMG):
                msg = Float64()
                msg.data = 0.0
                self.pub_gimbal[i].publish(msg)
                msg = Float64()
                msg.data = float(VARPI_NOM[i] * ramp)
                self.pub_wheel[i].publish(msg)
            return

        # Phase 2: PD + VSCMG
        self.auto_phase = 'control'
        target_rad = np.radians(self.target_euler_deg)
        q_d = euler_to_quat(target_rad[0], target_rad[1], target_rad[2])
        q_e = quat_error(self.q, q_d)

        tau_cmd = -2.0 * self.kp * q_e[1:4] - self.kd * self.omega

        gamma_dot, varpi_dot, m_sigma = allocate_vscmg(
            tau_cmd, self.gamma, self.varpi)
        self.m_sigma = m_sigma

        varpi_cmd = self.varpi + varpi_dot * self.dt
        varpi_cmd = np.clip(varpi_cmd, -VARPI_MAX, VARPI_MAX)

        for i in range(N_CMG):
            msg = Float64()
            msg.data = float(gamma_dot[i])
            self.pub_gimbal[i].publish(msg)
            msg = Float64()
            msg.data = float(varpi_cmd[i])
            self.pub_wheel[i].publish(msg)

    def _manual_control(self):
        for i in range(N_CMG):
            target_rad = math.radians(self.cmd_gimbal_deg[i])
            if self.joint_received:
                # Position loop: proportional control toward target angle
                error = target_rad - self.gamma[i]
                vel_cmd = GIMBAL_KP * error
                vel_cmd = max(-GAMMA_DOT_MAX, min(GAMMA_DOT_MAX, vel_cmd))
            else:
                # No feedback yet: use small velocity toward target
                vel_cmd = max(-GAMMA_DOT_MAX, min(GAMMA_DOT_MAX, target_rad))
            msg = Float64()
            msg.data = vel_cmd
            self.pub_gimbal[i].publish(msg)

        for i in range(N_CMG):
            msg = Float64()
            msg.data = self.cmd_rpm[i] * (2.0 * math.pi / 60.0)
            self.pub_wheel[i].publish(msg)


# ============================================================
# GUI
# ============================================================

def create_gui(node):
    BG = '#252530'
    FG = '#ffffff'
    FG_DIM = '#999999'
    CMG1_COLOR = '#ff6633'
    CMG2_COLOR = '#3399ff'
    AUTO_COLOR = '#33cc66'
    YAW_COLOR = '#ffcc00'
    WIDGET_BG = '#333340'
    TROUGH = '#1a1a24'
    FONT = ('Consolas', 10)
    FONT_B = ('Consolas', 10, 'bold')
    FONT_S = ('Consolas', 9)
    FONT_SB = ('Consolas', 9, 'bold')
    FONT_XS = ('Consolas', 8)

    root = tk.Tk()
    root.title('CubeSat CMG Commander')
    root.configure(bg=BG)
    root.geometry('560x850')
    root.resizable(False, False)

    # ==================== Status ====================
    frm_st = tk.LabelFrame(
        root, text='Status', bg=BG, fg=FG, font=FONT_B)
    frm_st.pack(fill='x', padx=8, pady=(8, 4))

    lbl_euler = tk.Label(
        frm_st, text='euler: ---', bg=BG, fg=FG,
        font=FONT, anchor='w')
    lbl_euler.pack(fill='x', padx=6, pady=1)
    lbl_omega = tk.Label(
        frm_st, text='body \u03c9: ---', bg=BG, fg=FG,
        font=FONT, anchor='w')
    lbl_omega.pack(fill='x', padx=6, pady=1)
    lbl_gamma = tk.Label(
        frm_st, text='gimbal: ---', bg=BG, fg=FG,
        font=FONT, anchor='w')
    lbl_gamma.pack(fill='x', padx=6, pady=1)
    lbl_wheel = tk.Label(
        frm_st, text='wheel: ---', bg=BG, fg=FG,
        font=FONT, anchor='w')
    lbl_wheel.pack(fill='x', padx=6, pady=1)
    lbl_ctrl = tk.Label(
        frm_st, text='att_err=---  m_\u03c3=---', bg=BG,
        fg=AUTO_COLOR, font=FONT, anchor='w')
    lbl_ctrl.pack(fill='x', padx=6, pady=(1, 4))

    # ==================== Mode ====================
    frm_mode = tk.Frame(root, bg=BG)
    frm_mode.pack(fill='x', padx=8, pady=4)

    var_mode = tk.StringVar(value='manual')
    tk.Radiobutton(
        frm_mode, text='Auto', variable=var_mode, value='auto',
        bg=BG, fg=AUTO_COLOR, selectcolor=WIDGET_BG, font=FONT_B,
        activebackground=BG, activeforeground=AUTO_COLOR,
    ).pack(side='left', padx=8)
    tk.Radiobutton(
        frm_mode, text='Manual', variable=var_mode, value='manual',
        bg=BG, fg=CMG1_COLOR, selectcolor=WIDGET_BG, font=FONT_B,
        activebackground=BG, activeforeground=CMG1_COLOR,
    ).pack(side='left', padx=8)

    lbl_phase = tk.Label(
        frm_mode, text='', bg=BG, fg=FG_DIM, font=FONT_S)
    lbl_phase.pack(side='right', padx=8)

    # ==================== Auto Control ====================
    frm_auto = tk.LabelFrame(
        root, text='Auto Control', bg=BG, fg=AUTO_COLOR, font=FONT_B)
    frm_auto.pack(fill='x', padx=8, pady=4)

    # --- Target ---
    frm_tgt = tk.Frame(frm_auto, bg=BG)
    frm_tgt.pack(fill='x', padx=6, pady=(4, 2))
    tk.Label(
        frm_tgt, text='Target:', bg=BG, fg=FG,
        font=FONT_S).pack(side='left')

    var_tgt_r = tk.DoubleVar(value=15.0)
    var_tgt_p = tk.DoubleVar(value=-10.0)
    var_tgt_y = tk.DoubleVar(value=25.0)

    for label, color, var in [
        ('R:', CMG1_COLOR, var_tgt_r),
        ('P:', CMG2_COLOR, var_tgt_p),
        ('Y:', YAW_COLOR, var_tgt_y),
    ]:
        tk.Label(
            frm_tgt, text=label, bg=BG, fg=color,
            font=FONT_SB).pack(side='left', padx=(8, 0))
        tk.Entry(
            frm_tgt, textvariable=var, bg=WIDGET_BG, fg=color,
            font=FONT_S, width=6, justify='center'
        ).pack(side='left', padx=2)
    tk.Label(
        frm_tgt, text='deg', bg=BG, fg=FG_DIM,
        font=FONT_XS).pack(side='left', padx=2)

    # --- Presets ---
    frm_pre = tk.Frame(frm_auto, bg=BG)
    frm_pre.pack(fill='x', padx=6, pady=2)

    def set_preset(r, p, y):
        var_tgt_r.set(r)
        var_tgt_p.set(p)
        var_tgt_y.set(y)

    for text, r, p, y in [
        ('M1: 15,-10,25', 15, -10, 25),
        ('M2: -20,15,-30', -20, 15, -30),
        ('M3: 10,20,-15', 10, 20, -15),
        ('Zero', 0, 0, 0),
    ]:
        tk.Button(
            frm_pre, text=text,
            command=lambda r=r, p=p, y=y: set_preset(r, p, y),
            bg=WIDGET_BG, fg=FG, font=FONT_XS,
            activebackground='#555565', activeforeground=FG,
            padx=4, pady=1,
        ).pack(side='left', padx=2)

    # --- Gains: Kp ---
    tk.Label(
        frm_auto, text='Kp (proportional)', bg=BG,
        fg=FG_DIM, font=FONT_XS).pack(fill='x', padx=6, pady=(6, 0))

    axis_info = [('R', CMG1_COLOR), ('P', CMG2_COLOR), ('Y', YAW_COLOR)]
    kp_defaults = [0.8, 0.8, 15.0]
    kp_ranges = [(0, 5.0), (0, 5.0), (0, 30.0)]
    var_kp = []

    for (lbl, color), default, (lo, hi) in zip(
            axis_info, kp_defaults, kp_ranges):
        frm = tk.Frame(frm_auto, bg=BG)
        frm.pack(fill='x', padx=6, pady=1)
        tk.Label(
            frm, text=f'{lbl}:', bg=BG, fg=color,
            font=FONT_SB, width=2).pack(side='left')
        v = tk.DoubleVar(value=default)
        var_kp.append(v)
        tk.Scale(
            frm, from_=lo, to=hi, orient='horizontal', variable=v,
            bg=WIDGET_BG, fg=color, troughcolor=TROUGH,
            highlightthickness=0, font=FONT_XS, length=340,
            resolution=0.05, showvalue=False,
        ).pack(side='left', padx=4)
        tk.Entry(
            frm, textvariable=v, bg=WIDGET_BG, fg=color,
            font=FONT_S, width=6, justify='center',
        ).pack(side='left', padx=2)

    # --- Gains: Kd ---
    tk.Label(
        frm_auto, text='Kd (derivative)', bg=BG,
        fg=FG_DIM, font=FONT_XS).pack(fill='x', padx=6, pady=(4, 0))

    kd_defaults = [5.5, 5.5, 0.5]
    kd_ranges = [(0, 15.0), (0, 15.0), (0, 5.0)]
    var_kd = []

    for (lbl, color), default, (lo, hi) in zip(
            axis_info, kd_defaults, kd_ranges):
        frm = tk.Frame(frm_auto, bg=BG)
        frm.pack(fill='x', padx=6, pady=1)
        tk.Label(
            frm, text=f'{lbl}:', bg=BG, fg=color,
            font=FONT_SB, width=2).pack(side='left')
        v = tk.DoubleVar(value=default)
        var_kd.append(v)
        tk.Scale(
            frm, from_=lo, to=hi, orient='horizontal', variable=v,
            bg=WIDGET_BG, fg=color, troughcolor=TROUGH,
            highlightthickness=0, font=FONT_XS, length=340,
            resolution=0.05, showvalue=False,
        ).pack(side='left', padx=4)
        tk.Entry(
            frm, textvariable=v, bg=WIDGET_BG, fg=color,
            font=FONT_S, width=6, justify='center',
        ).pack(side='left', padx=2)

    tk.Frame(frm_auto, bg=BG, height=4).pack()

    # ==================== Manual Control ====================
    frm_man = tk.LabelFrame(
        root, text='Manual Control', bg=BG, fg=CMG1_COLOR, font=FONT_B)
    frm_man.pack(fill='x', padx=8, pady=4)

    # --- Gimbal ---
    frm_go = tk.Frame(frm_man, bg=BG)
    frm_go.pack(fill='x', padx=6, pady=(4, 0))
    tk.Label(
        frm_go, text='Gimbal', bg=BG, fg=FG,
        font=FONT_SB).pack(side='left')
    var_sync_g = tk.BooleanVar(value=False)
    var_opp_g = tk.BooleanVar(value=False)
    tk.Checkbutton(
        frm_go, text='Sync', variable=var_sync_g,
        bg=BG, fg=FG, selectcolor=WIDGET_BG, font=FONT_XS,
        activebackground=BG, activeforeground=FG,
    ).pack(side='left', padx=4)
    tk.Checkbutton(
        frm_go, text='\u00b1', variable=var_opp_g,
        bg=BG, fg=YAW_COLOR, selectcolor=WIDGET_BG, font=FONT_XS,
        activebackground=BG, activeforeground=YAW_COLOR,
    ).pack(side='left', padx=2)

    var_g1 = tk.DoubleVar(value=0.0)
    var_g2 = tk.DoubleVar(value=0.0)

    for label, color, var in [
        ('\u03b31:', CMG1_COLOR, var_g1),
        ('\u03b32:', CMG2_COLOR, var_g2),
    ]:
        frm = tk.Frame(frm_man, bg=BG)
        frm.pack(fill='x', padx=6, pady=1)
        tk.Label(
            frm, text=label, bg=BG, fg=color,
            font=FONT_SB, width=3).pack(side='left')
        tk.Scale(
            frm, from_=-90, to=90, orient='horizontal', variable=var,
            bg=WIDGET_BG, fg=color, troughcolor=TROUGH,
            highlightthickness=0, font=FONT_XS, length=340,
            resolution=0.1, showvalue=False,
        ).pack(side='left', padx=4)
        tk.Entry(
            frm, textvariable=var, bg=WIDGET_BG, fg=color,
            font=FONT_S, width=7, justify='center',
        ).pack(side='left', padx=2)

    # --- Wheel ---
    frm_wo = tk.Frame(frm_man, bg=BG)
    frm_wo.pack(fill='x', padx=6, pady=(6, 0))
    tk.Label(
        frm_wo, text='Wheel', bg=BG, fg=FG,
        font=FONT_SB).pack(side='left')
    var_sync_w = tk.BooleanVar(value=False)
    var_opp_w = tk.BooleanVar(value=True)
    tk.Checkbutton(
        frm_wo, text='Sync', variable=var_sync_w,
        bg=BG, fg=FG, selectcolor=WIDGET_BG, font=FONT_XS,
        activebackground=BG, activeforeground=FG,
    ).pack(side='left', padx=4)
    tk.Checkbutton(
        frm_wo, text='\u00b1', variable=var_opp_w,
        bg=BG, fg=YAW_COLOR, selectcolor=WIDGET_BG, font=FONT_XS,
        activebackground=BG, activeforeground=YAW_COLOR,
    ).pack(side='left', padx=2)

    var_w1 = tk.DoubleVar(value=0.0)
    var_w2 = tk.DoubleVar(value=0.0)

    for label, color, var in [
        ('\u03d61:', CMG1_COLOR, var_w1),
        ('\u03d62:', CMG2_COLOR, var_w2),
    ]:
        frm = tk.Frame(frm_man, bg=BG)
        frm.pack(fill='x', padx=6, pady=1)
        tk.Label(
            frm, text=label, bg=BG, fg=color,
            font=FONT_SB, width=3).pack(side='left')
        tk.Scale(
            frm, from_=-20000, to=20000, orient='horizontal', variable=var,
            bg=WIDGET_BG, fg=color, troughcolor=TROUGH,
            highlightthickness=0, font=FONT_XS, length=340,
            resolution=100, showvalue=False,
        ).pack(side='left', padx=4)
        tk.Entry(
            frm, textvariable=var, bg=WIDGET_BG, fg=color,
            font=FONT_S, width=7, justify='center',
        ).pack(side='left', padx=2)

    tk.Frame(frm_man, bg=BG, height=4).pack()

    # ==================== Reset ====================
    frm_btn = tk.Frame(root, bg=BG)
    frm_btn.pack(fill='x', padx=8, pady=6)

    def reset_all():
        var_g1.set(0.0)
        var_g2.set(0.0)
        var_w1.set(0.0)
        var_w2.set(0.0)
        for i, v in enumerate(var_kp):
            v.set(kp_defaults[i])
        for i, v in enumerate(var_kd):
            v.set(kd_defaults[i])
        var_tgt_r.set(15.0)
        var_tgt_p.set(-10.0)
        var_tgt_y.set(25.0)

    tk.Button(
        frm_btn, text='Reset (R)', command=reset_all,
        bg='#444450', fg=FG, font=FONT_B,
        activebackground='#555565', activeforeground=FG,
        width=12,
    ).pack(side='left', padx=4)

    tk.Label(
        frm_btn, text='kinematic velocity', bg=BG,
        fg=FG_DIM, font=FONT_XS).pack(side='right', padx=4)

    root.bind('r', lambda e: reset_all())

    # ==================== Sync logic ====================
    _upd = [False]

    def on_g1(*_):
        if _upd[0]:
            return
        if var_sync_g.get():
            _upd[0] = True
            s = -1.0 if var_opp_g.get() else 1.0
            var_g2.set(round(var_g1.get() * s, 1))
            _upd[0] = False

    def on_g2(*_):
        if _upd[0]:
            return
        if var_sync_g.get():
            _upd[0] = True
            s = -1.0 if var_opp_g.get() else 1.0
            var_g1.set(round(var_g2.get() * s, 1))
            _upd[0] = False

    def on_w1(*_):
        if _upd[0]:
            return
        if var_sync_w.get():
            _upd[0] = True
            s = -1.0 if var_opp_w.get() else 1.0
            var_w2.set(round(var_w1.get() * s, 1))
            _upd[0] = False

    def on_w2(*_):
        if _upd[0]:
            return
        if var_sync_w.get():
            _upd[0] = True
            s = -1.0 if var_opp_w.get() else 1.0
            var_w1.set(round(var_w2.get() * s, 1))
            _upd[0] = False

    var_g1.trace_add('write', on_g1)
    var_g2.trace_add('write', on_g2)
    var_w1.trace_add('write', on_w1)
    var_w2.trace_add('write', on_w2)

    # ==================== Mode switch ====================
    prev_mode = ['manual']

    def on_mode_change(*_):
        new_mode = var_mode.get()
        if new_mode == 'manual' and prev_mode[0] == 'auto':
            # Snap sliders to current actual values
            var_g1.set(round(math.degrees(node.gamma[0]), 1))
            var_g2.set(round(math.degrees(node.gamma[1]), 1))
            rpm1 = node.varpi[0] * 60.0 / (2.0 * math.pi)
            rpm2 = node.varpi[1] * 60.0 / (2.0 * math.pi)
            var_w1.set(round(rpm1 / 100) * 100)
            var_w2.set(round(rpm2 / 100) * 100)
        prev_mode[0] = new_mode

    var_mode.trace_add('write', on_mode_change)

    # ==================== Periodic update (10 Hz) ====================
    def update():
        # Push GUI values to node
        node.mode = var_mode.get()
        node.cmd_gimbal_deg[0] = var_g1.get()
        node.cmd_gimbal_deg[1] = var_g2.get()
        node.cmd_rpm[0] = var_w1.get()
        node.cmd_rpm[1] = var_w2.get()

        try:
            node.target_euler_deg = np.array([
                var_tgt_r.get(), var_tgt_p.get(), var_tgt_y.get()])
        except (tk.TclError, ValueError):
            pass

        try:
            node.kp = np.array([v.get() for v in var_kp])
            node.kd = np.array([v.get() for v in var_kd])
        except (tk.TclError, ValueError):
            pass

        # Update status
        euler_deg = np.degrees(quat_to_euler(node.q))
        lbl_euler.config(
            text=f'euler: R={euler_deg[0]:+6.1f}\u00b0 '
                 f'P={euler_deg[1]:+6.1f}\u00b0 '
                 f'Y={euler_deg[2]:+6.1f}\u00b0')
        ox, oy, oz = node.omega
        lbl_omega.config(
            text=f'body \u03c9: [{ox:+.4f}, {oy:+.4f}, {oz:+.4f}] rad/s')
        g1d = math.degrees(node.gamma[0])
        g2d = math.degrees(node.gamma[1])
        lbl_gamma.config(
            text=f'gimbal: \u03b31={g1d:+6.1f}\u00b0  '
                 f'\u03b32={g2d:+6.1f}\u00b0')
        rpm1 = node.varpi[0] * 60.0 / (2.0 * math.pi)
        rpm2 = node.varpi[1] * 60.0 / (2.0 * math.pi)
        lbl_wheel.config(
            text=f'wheel:  \u03d61={rpm1:+7.0f}  '
                 f'\u03d62={rpm2:+7.0f} RPM')
        lbl_ctrl.config(
            text=f'att_err={node.att_err_deg:.2f}\u00b0  '
                 f'm_\u03c3={node.m_sigma:.4f}')

        # Phase indicator
        if node.mode == 'auto':
            if node.auto_phase == 'spinup':
                lbl_phase.config(text='Spinning up...', fg=YAW_COLOR)
            elif node.auto_phase == 'control':
                lbl_phase.config(text='Control active', fg=AUTO_COLOR)
            else:
                lbl_phase.config(text='Idle', fg=FG_DIM)
        else:
            lbl_phase.config(text='Manual', fg=CMG1_COLOR)

        root.after(100, update)

    root.after(500, update)
    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CubeSatCMGCommander()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        create_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
