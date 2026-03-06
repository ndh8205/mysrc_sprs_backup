"""
CubeSat VSCMG Attitude Controller (Joint velocity commands)
- MATLAB cube_sat_hanul 시뮬레이션과 동일한 PD + VSCMG 할당
- CMG 동역학: Gazebo DART 물리엔진 (physical joints, kinematic velocity)
- 컨트롤러: gimbal velocity + wheel velocity 명령 출력
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Float64MultiArray
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
    """q_e = q_d^{-1} * q  (scalar-first)"""
    q_e = quat_mult(quat_conj(q_d), q)
    if q_e[0] < 0:
        q_e = -q_e
    return q_e


def euler_to_quat(roll, pitch, yaw):
    """ZYX Euler (rad) -> scalar-first quaternion [w,x,y,z]"""
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
    """scalar-first quaternion -> [roll, pitch, yaw] (rad)"""
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
# VSCMG parameters (from MATLAB CMG_sim.m)
# ============================================================

N_CMG = 2
I_DB = np.array([3.65e-5, 3.65e-5])   # wheel spin inertia [kg*m^2]

# CMG layout: 2x parallel, gimbal axis Y, spin axis X
GIMBAL_AXIS = np.array([0.0, 1.0, 0.0])
SPIN_AXIS_0 = np.array([1.0, 0.0, 0.0])

# Nominal wheel speed: +/-10000 RPM
VARPI_NOM = np.array([1.0, -1.0]) * 10000.0 * (2.0 * np.pi / 60.0)
VARPI_MAX = 20000.0 * (2.0 * np.pi / 60.0)
GAMMA_DOT_MAX = 1.0  # rad/s

# PD gains (MATLAB CMG_sim.m 원본)
KP = np.array([0.8, 0.8, 15.0])
KD = np.array([5.5, 5.5, 0.5])

# SRI parameters
SRI_LAMBDA0 = 0.01
SRI_LAMBDA_MAX = 0.5
SRI_M_THRESHOLD = 0.01

# VSCMG allocation
W_RW = 0.15          # wheel weight
K_NULL_VARPI = 0.01  # null-space recovery gain

# Maneuver sequence (MATLAB batch.seq)
MANEUVERS = [
    (0.0,   np.radians([15.0, -10.0, 25.0])),
    (130.0, np.radians([-20.0, 15.0, -30.0])),
    (260.0, np.radians([10.0, 20.0, -15.0])),
]
TOTAL_TIME = 390.0

# Wheel spin-up
SPINUP_DURATION = 5.0  # seconds to ramp wheels to nominal


# ============================================================
# VSCMG Jacobian & Allocator
# ============================================================

def get_spin_axis(gamma_i):
    """s = cos(g)*s0 + sin(g)*(a x s0)"""
    a_cross_s0 = np.cross(GIMBAL_AXIS, SPIN_AXIS_0)
    return np.cos(gamma_i) * SPIN_AXIS_0 + np.sin(gamma_i) * a_cross_s0


def get_torque_axis(gamma_i):
    """t = a x s"""
    return np.cross(GIMBAL_AXIS, get_spin_axis(gamma_i))


def get_jacobian(gamma, varpi):
    """VSCMG Jacobian: A (CMG) and B (RW)"""
    A = np.zeros((3, N_CMG))
    B = np.zeros((3, N_CMG))
    for i in range(N_CMG):
        s_i = get_spin_axis(gamma[i])
        t_i = get_torque_axis(gamma[i])
        A[:, i] = I_DB[i] * varpi[i] * t_i
        B[:, i] = I_DB[i] * s_i
    return A, B


def allocate_vscmg(tau_cmd, gamma, varpi):
    """Weighted SRI VSCMG allocation (MATLAB Allocator.m)"""
    A, B_rw = get_jacobian(gamma, varpi)

    W_cmg = A @ A.T
    m_sigma = np.sqrt(max(np.linalg.det(W_cmg), 0.0))

    if m_sigma < SRI_M_THRESHOLD:
        lam = SRI_LAMBDA0 + (1.0 - m_sigma / SRI_M_THRESHOLD) ** 2 * SRI_LAMBDA_MAX
    else:
        lam = SRI_LAMBDA0

    w2_inv = np.diag(np.concatenate([
        np.ones(N_CMG),
        np.ones(N_CMG) / (W_RW ** 2),
    ]))
    D = np.hstack([A, B_rw])

    DW = D @ w2_inv
    DWDt = DW @ D.T + lam ** 2 * np.eye(3)
    D_sri = w2_inv @ D.T @ np.linalg.solve(DWDt, np.eye(3))
    u = -D_sri @ tau_cmd

    N_null = np.eye(2 * N_CMG) - D_sri @ D
    u_null_raw = np.concatenate([
        np.zeros(N_CMG),
        K_NULL_VARPI * (VARPI_NOM - varpi),
    ])
    u += N_null @ u_null_raw

    gamma_dot = np.clip(u[:N_CMG], -GAMMA_DOT_MAX, GAMMA_DOT_MAX)
    varpi_dot = np.clip(u[N_CMG:], -500.0, 500.0)

    return gamma_dot, varpi_dot, m_sigma


# ============================================================
# ROS 2 Node
# ============================================================

class CubeSatCMGController(Node):
    def __init__(self):
        super().__init__('cubesat_cmg_controller')

        # Body state (from IMU)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # scalar-first
        self.omega = np.zeros(3)  # body-frame angular velocity
        self.imu_received = False

        # CMG state (from joint_states)
        self.gamma = np.zeros(N_CMG)       # gimbal angles [rad]
        self.varpi = np.zeros(N_CMG)       # wheel speeds [rad/s]
        self.joint_received = False

        # Target wheel velocity (ramped up during spin-up, then from allocator)
        self.varpi_cmd = np.zeros(N_CMG)

        # Control
        self.start_time = None
        self.last_log_time = -999.0
        euler_d = MANEUVERS[0][1]
        self.q_d = euler_to_quat(euler_d[0], euler_d[1], euler_d[2])

        # Subscribers
        self.create_subscription(
            Imu, '/cubesat_cmg/imu', self.imu_callback, 10)
        self.create_subscription(
            JointState, '/cubesat_cmg/joint_states', self.joint_callback, 10)

        # Publishers: joint velocity commands
        self.pub_gimbal = [
            self.create_publisher(Float64, '/cubesat_cmg/cmg1_gimbal/cmd_vel', 10),
            self.create_publisher(Float64, '/cubesat_cmg/cmg2_gimbal/cmd_vel', 10),
        ]
        self.pub_wheel = [
            self.create_publisher(Float64, '/cubesat_cmg/cmg1_wheel/cmd_vel', 10),
            self.create_publisher(Float64, '/cubesat_cmg/cmg2_wheel/cmd_vel', 10),
        ]

        # Publisher: telemetry
        self.pub_log = self.create_publisher(
            Float64MultiArray, '/cubesat_cmg/attitude_log', 10)

        # Control timer (100Hz = dt 0.01s)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            'CubeSat CMG Controller started (PD + VSCMG joints, 100Hz)')

    def imu_callback(self, msg):
        # ROS (x,y,z,w) -> internal scalar-first (w,x,y,z)
        o = msg.orientation
        self.q = np.array([o.w, o.x, o.y, o.z])
        norm_q = np.linalg.norm(self.q)
        if norm_q > 0:
            self.q /= norm_q

        av = msg.angular_velocity
        self.omega = np.array([av.x, av.y, av.z])
        self.imu_received = True

    def joint_callback(self, msg):
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
        if not self.imu_received or not self.joint_received:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info(
                f'Control started. q=[{self.q[0]:.4f},{self.q[1]:.4f},'
                f'{self.q[2]:.4f},{self.q[3]:.4f}]')

        sim_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if sim_time > TOTAL_TIME:
            self.get_logger().info('Simulation complete (390s)')
            self.timer.cancel()
            return

        # ---- Phase 1: Wheel spin-up (ramp to nominal) ----
        if sim_time < SPINUP_DURATION:
            ramp = sim_time / SPINUP_DURATION
            self.varpi_cmd = VARPI_NOM * ramp

            # No gimbal motion during spin-up
            for i in range(N_CMG):
                msg = Float64()
                msg.data = 0.0
                self.pub_gimbal[i].publish(msg)

                msg = Float64()
                msg.data = float(self.varpi_cmd[i])
                self.pub_wheel[i].publish(msg)

            # Log during spin-up
            if sim_time - self.last_log_time >= 1.0:
                self.last_log_time = sim_time
                rpm1 = self.varpi[0] * 60.0 / (2.0 * np.pi)
                rpm2 = self.varpi[1] * 60.0 / (2.0 * np.pi)
                self.get_logger().info(
                    f't={sim_time:.1f}s  [spin-up] '
                    f'wheel=[{rpm1:.0f},{rpm2:.0f}] RPM  '
                    f'target=[{VARPI_NOM[0]*60/(2*np.pi):.0f},'
                    f'{VARPI_NOM[1]*60/(2*np.pi):.0f}]')
            return

        # ---- Phase 2: PD Control + VSCMG Allocation ----
        ctrl_time = sim_time - SPINUP_DURATION

        # Update target quaternion
        for t_start, euler_d in MANEUVERS:
            if ctrl_time >= t_start:
                self.q_d = euler_to_quat(euler_d[0], euler_d[1], euler_d[2])

        # PD Control
        q_e = quat_error(self.q, self.q_d)
        tau_cmd = -2.0 * KP * q_e[1:4] - KD * self.omega

        # VSCMG Allocation (uses actual gamma/varpi from joint_states)
        gamma_dot, varpi_dot, m_sigma = allocate_vscmg(
            tau_cmd, self.gamma, self.varpi)

        # Update target wheel velocity
        self.varpi_cmd = self.varpi + varpi_dot * self.dt
        self.varpi_cmd = np.clip(self.varpi_cmd, -VARPI_MAX, VARPI_MAX)

        # Publish gimbal velocity commands
        for i in range(N_CMG):
            msg = Float64()
            msg.data = float(gamma_dot[i])
            self.pub_gimbal[i].publish(msg)

        # Publish wheel velocity commands
        for i in range(N_CMG):
            msg = Float64()
            msg.data = float(self.varpi_cmd[i])
            self.pub_wheel[i].publish(msg)

        # ---- Telemetry ----
        att_err_deg = 2.0 * np.arccos(
            np.clip(abs(q_e[0]), 0.0, 1.0)) * 180.0 / np.pi
        euler_deg = np.degrees(quat_to_euler(self.q))

        log_msg = Float64MultiArray()
        log_msg.data = [
            sim_time,
            self.q[0], self.q[1], self.q[2], self.q[3],
            self.omega[0], self.omega[1], self.omega[2],
            self.gamma[0], self.gamma[1],
            self.varpi[0], self.varpi[1],
            att_err_deg, m_sigma,
            euler_deg[0], euler_deg[1], euler_deg[2],
            tau_cmd[0], tau_cmd[1], tau_cmd[2],
        ]
        self.pub_log.publish(log_msg)

        # Periodic logging (every 5 seconds)
        if sim_time - self.last_log_time >= 5.0:
            self.last_log_time = sim_time
            rpm1 = self.varpi[0] * 60.0 / (2.0 * np.pi)
            rpm2 = self.varpi[1] * 60.0 / (2.0 * np.pi)
            self.get_logger().info(
                f't={sim_time:.1f}s  att_err={att_err_deg:.2f}deg  '
                f'euler=[{euler_deg[0]:.1f},{euler_deg[1]:.1f},{euler_deg[2]:.1f}]  '
                f'gamma=[{np.degrees(self.gamma[0]):.1f},'
                f'{np.degrees(self.gamma[1]):.1f}]deg  '
                f'wheel=[{rpm1:.0f},{rpm2:.0f}]RPM  '
                f'm={m_sigma:.4f}')


def main(args=None):
    rclpy.init(args=args)
    node = CubeSatCMGController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
