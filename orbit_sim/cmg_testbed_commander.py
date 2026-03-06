"""
CMG Testbed GUI Commander
- MATLAB CMG_visualize.m UI 스타일 재현
- tkinter 슬라이더로 짐벌 각도(deg) + 휠 RPM 제어
- Sync/Opposite 체크박스
- 짐벌: 속도제어 + 소프트웨어 위치루프 (gimbal angle → velocity cmd)
- 휠: 직접 속도 명령 (RPM → rad/s)
"""

import threading
import math
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, JointState


# MATLAB initial values
VARPI_NOM_RPM = [10000.0, -10000.0]
GAMMA_DOT_MAX = 1.0  # rad/s


class CMGTestbedNode(Node):
    def __init__(self):
        super().__init__('cmg_testbed_commander')

        # Publishers
        self.pub_gimbal = [
            self.create_publisher(Float64, '/cmg_testbed/cmg1_gimbal/cmd_vel', 10),
            self.create_publisher(Float64, '/cmg_testbed/cmg2_gimbal/cmd_vel', 10),
        ]
        self.pub_wheel = [
            self.create_publisher(Float64, '/cmg_testbed/cmg1_wheel/cmd_vel', 10),
            self.create_publisher(Float64, '/cmg_testbed/cmg2_wheel/cmd_vel', 10),
        ]

        # Subscribers
        self.create_subscription(
            JointState, '/cmg_testbed/joint_states', self.joint_cb, 10)
        self.create_subscription(
            Imu, '/cmg_testbed/imu', self.imu_cb, 10)

        # State (thread-safe via GIL for simple reads/writes)
        self.gimbal_pos = [0.0, 0.0]
        self.wheel_vel_rads = [0.0, 0.0]
        self.body_omega = [0.0, 0.0, 0.0]

        # Commands (start at 0, user ramps up via GUI to avoid spin-up reaction)
        self.cmd_gimbal_deg = [0.0, 0.0]
        self.cmd_rpm = [0.0, 0.0]

        # Position control gain for gimbal
        self.gimbal_kp = 5.0

        # Timer (100 Hz)
        self.create_timer(0.01, self.publish_commands)
        self.last_log_time = -999.0
        self.get_logger().info('CMG Testbed Commander started')

    def joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'cmg1_gimbal_joint' and len(msg.position) > i:
                self.gimbal_pos[0] = msg.position[i]
            elif name == 'cmg2_gimbal_joint' and len(msg.position) > i:
                self.gimbal_pos[1] = msg.position[i]
            if name == 'cmg1_wheel_joint' and len(msg.velocity) > i:
                self.wheel_vel_rads[0] = msg.velocity[i]
            elif name == 'cmg2_wheel_joint' and len(msg.velocity) > i:
                self.wheel_vel_rads[1] = msg.velocity[i]

    def imu_cb(self, msg):
        self.body_omega = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]

    def publish_commands(self):
        # Gimbal: software position loop → velocity command
        for i in range(2):
            target_rad = math.radians(self.cmd_gimbal_deg[i])
            error = target_rad - self.gimbal_pos[i]
            vel_cmd = self.gimbal_kp * error
            vel_cmd = max(-GAMMA_DOT_MAX, min(GAMMA_DOT_MAX, vel_cmd))
            msg = Float64()
            msg.data = vel_cmd
            self.pub_gimbal[i].publish(msg)

        # Wheel: direct velocity command (RPM → rad/s)
        for i in range(2):
            msg = Float64()
            msg.data = self.cmd_rpm[i] * (2.0 * math.pi / 60.0)
            self.pub_wheel[i].publish(msg)


def create_gui(node):
    BG = '#252530'
    FG = '#ffffff'
    FG_DIM = '#999999'
    CMG1_COLOR = '#ff6633'
    CMG2_COLOR = '#3399ff'

    root = tk.Tk()
    root.title('CMG Testbed Commander')
    root.configure(bg=BG)
    root.geometry('520x620')
    root.resizable(False, False)

    # --- Status section ---
    frm_status = tk.LabelFrame(
        root, text='Status', bg=BG, fg=FG, font=('Consolas', 10, 'bold'))
    frm_status.pack(fill='x', padx=8, pady=(8, 4))

    lbl_omega = tk.Label(
        frm_status, text='body omega: ---', bg=BG, fg=FG,
        font=('Consolas', 10), anchor='w')
    lbl_omega.pack(fill='x', padx=6, pady=2)

    lbl_gamma = tk.Label(
        frm_status, text='gimbal act: ---', bg=BG, fg=FG,
        font=('Consolas', 10), anchor='w')
    lbl_gamma.pack(fill='x', padx=6, pady=2)

    lbl_rpm = tk.Label(
        frm_status, text='wheel act:  ---', bg=BG, fg=FG,
        font=('Consolas', 10), anchor='w')
    lbl_rpm.pack(fill='x', padx=6, pady=(2, 6))

    # --- Gimbal section ---
    frm_gimbal = tk.LabelFrame(
        root, text='Gimbal angle [deg]', bg=BG, fg=CMG1_COLOR,
        font=('Consolas', 10, 'bold'))
    frm_gimbal.pack(fill='x', padx=8, pady=4)

    # Sync/Opposite checkboxes
    frm_gimbal_opt = tk.Frame(frm_gimbal, bg=BG)
    frm_gimbal_opt.pack(fill='x', padx=6, pady=(4, 0))
    var_sync_g = tk.BooleanVar(value=False)
    var_opp_g = tk.BooleanVar(value=False)
    tk.Checkbutton(
        frm_gimbal_opt, text='Sync', variable=var_sync_g,
        bg=BG, fg=FG, selectcolor='#333340', font=('Consolas', 9),
        activebackground=BG, activeforeground=FG
    ).pack(side='left', padx=4)
    tk.Checkbutton(
        frm_gimbal_opt, text='\u00b1 (Opposite)', variable=var_opp_g,
        bg=BG, fg='#ffcc00', selectcolor='#333340', font=('Consolas', 9),
        activebackground=BG, activeforeground='#ffcc00'
    ).pack(side='left', padx=4)

    # gamma1
    frm_g1 = tk.Frame(frm_gimbal, bg=BG)
    frm_g1.pack(fill='x', padx=6, pady=2)
    tk.Label(frm_g1, text='\u03b31:', bg=BG, fg=CMG1_COLOR,
             font=('Consolas', 10, 'bold'), width=3).pack(side='left')
    var_g1 = tk.DoubleVar(value=0.0)
    sld_g1 = tk.Scale(
        frm_g1, from_=-90, to=90, orient='horizontal', variable=var_g1,
        bg='#333340', fg=CMG1_COLOR, troughcolor='#1a1a24',
        highlightthickness=0, font=('Consolas', 8), length=340,
        resolution=0.1, showvalue=False)
    sld_g1.pack(side='left', padx=4)
    ent_g1 = tk.Entry(
        frm_g1, textvariable=var_g1, bg='#333340', fg=CMG1_COLOR,
        font=('Consolas', 10), width=7, justify='center')
    ent_g1.pack(side='left', padx=4)

    # gamma2
    frm_g2 = tk.Frame(frm_gimbal, bg=BG)
    frm_g2.pack(fill='x', padx=6, pady=(2, 6))
    tk.Label(frm_g2, text='\u03b32:', bg=BG, fg=CMG2_COLOR,
             font=('Consolas', 10, 'bold'), width=3).pack(side='left')
    var_g2 = tk.DoubleVar(value=0.0)
    sld_g2 = tk.Scale(
        frm_g2, from_=-90, to=90, orient='horizontal', variable=var_g2,
        bg='#333340', fg=CMG2_COLOR, troughcolor='#1a1a24',
        highlightthickness=0, font=('Consolas', 8), length=340,
        resolution=0.1, showvalue=False)
    sld_g2.pack(side='left', padx=4)
    ent_g2 = tk.Entry(
        frm_g2, textvariable=var_g2, bg='#333340', fg=CMG2_COLOR,
        font=('Consolas', 10), width=7, justify='center')
    ent_g2.pack(side='left', padx=4)

    # --- Wheel section ---
    frm_wheel = tk.LabelFrame(
        root, text='Wheel speed [RPM]', bg=BG, fg=CMG2_COLOR,
        font=('Consolas', 10, 'bold'))
    frm_wheel.pack(fill='x', padx=8, pady=4)

    # Sync/Opposite checkboxes
    frm_wheel_opt = tk.Frame(frm_wheel, bg=BG)
    frm_wheel_opt.pack(fill='x', padx=6, pady=(4, 0))
    var_sync_w = tk.BooleanVar(value=False)
    var_opp_w = tk.BooleanVar(value=True)
    tk.Checkbutton(
        frm_wheel_opt, text='Sync', variable=var_sync_w,
        bg=BG, fg=FG, selectcolor='#333340', font=('Consolas', 9),
        activebackground=BG, activeforeground=FG
    ).pack(side='left', padx=4)
    tk.Checkbutton(
        frm_wheel_opt, text='\u00b1 (Opposite)', variable=var_opp_w,
        bg=BG, fg='#ffcc00', selectcolor='#333340', font=('Consolas', 9),
        activebackground=BG, activeforeground='#ffcc00'
    ).pack(side='left', padx=4)

    # rpm1
    frm_w1 = tk.Frame(frm_wheel, bg=BG)
    frm_w1.pack(fill='x', padx=6, pady=2)
    tk.Label(frm_w1, text='\u03d61:', bg=BG, fg=CMG1_COLOR,
             font=('Consolas', 10, 'bold'), width=3).pack(side='left')
    var_w1 = tk.DoubleVar(value=0.0)
    sld_w1 = tk.Scale(
        frm_w1, from_=-20000, to=20000, orient='horizontal', variable=var_w1,
        bg='#333340', fg=CMG1_COLOR, troughcolor='#1a1a24',
        highlightthickness=0, font=('Consolas', 8), length=340,
        resolution=100, showvalue=False)
    sld_w1.pack(side='left', padx=4)
    ent_w1 = tk.Entry(
        frm_w1, textvariable=var_w1, bg='#333340', fg=CMG1_COLOR,
        font=('Consolas', 10), width=7, justify='center')
    ent_w1.pack(side='left', padx=4)

    # rpm2
    frm_w2 = tk.Frame(frm_wheel, bg=BG)
    frm_w2.pack(fill='x', padx=6, pady=(2, 6))
    tk.Label(frm_w2, text='\u03d62:', bg=BG, fg=CMG2_COLOR,
             font=('Consolas', 10, 'bold'), width=3).pack(side='left')
    var_w2 = tk.DoubleVar(value=0.0)
    sld_w2 = tk.Scale(
        frm_w2, from_=-20000, to=20000, orient='horizontal', variable=var_w2,
        bg='#333340', fg=CMG2_COLOR, troughcolor='#1a1a24',
        highlightthickness=0, font=('Consolas', 8), length=340,
        resolution=100, showvalue=False)
    sld_w2.pack(side='left', padx=4)
    ent_w2 = tk.Entry(
        frm_w2, textvariable=var_w2, bg='#333340', fg=CMG2_COLOR,
        font=('Consolas', 10), width=7, justify='center')
    ent_w2.pack(side='left', padx=4)

    # --- Reset button ---
    frm_btn = tk.Frame(root, bg=BG)
    frm_btn.pack(fill='x', padx=8, pady=8)

    def reset_all():
        var_g1.set(0.0)
        var_g2.set(0.0)
        var_w1.set(0.0)
        var_w2.set(0.0)

    tk.Button(
        frm_btn, text='Reset (R)', command=reset_all,
        bg='#444450', fg=FG, font=('Consolas', 10, 'bold'),
        activebackground='#555565', activeforeground=FG,
        width=12
    ).pack(side='left', padx=4)

    lbl_info = tk.Label(
        frm_btn, text='use_force_commands=false (kinematic)',
        bg=BG, fg=FG_DIM, font=('Consolas', 8))
    lbl_info.pack(side='right', padx=4)

    # Keyboard shortcut
    root.bind('r', lambda e: reset_all())

    # Sync logic
    _updating = [False]

    def on_g1_change(*_):
        if _updating[0]:
            return
        if var_sync_g.get():
            _updating[0] = True
            sign = -1.0 if var_opp_g.get() else 1.0
            var_g2.set(round(var_g1.get() * sign, 1))
            _updating[0] = False

    def on_g2_change(*_):
        if _updating[0]:
            return
        if var_sync_g.get():
            _updating[0] = True
            sign = -1.0 if var_opp_g.get() else 1.0
            var_g1.set(round(var_g2.get() * sign, 1))
            _updating[0] = False

    def on_w1_change(*_):
        if _updating[0]:
            return
        if var_sync_w.get():
            _updating[0] = True
            sign = -1.0 if var_opp_w.get() else 1.0
            var_w2.set(round(var_w1.get() * sign, 1))
            _updating[0] = False

    def on_w2_change(*_):
        if _updating[0]:
            return
        if var_sync_w.get():
            _updating[0] = True
            sign = -1.0 if var_opp_w.get() else 1.0
            var_w1.set(round(var_w2.get() * sign, 1))
            _updating[0] = False

    var_g1.trace_add('write', on_g1_change)
    var_g2.trace_add('write', on_g2_change)
    var_w1.trace_add('write', on_w1_change)
    var_w2.trace_add('write', on_w2_change)

    # Periodic update (10 Hz)
    def update():
        # Push slider values to ROS node
        node.cmd_gimbal_deg[0] = var_g1.get()
        node.cmd_gimbal_deg[1] = var_g2.get()
        node.cmd_rpm[0] = var_w1.get()
        node.cmd_rpm[1] = var_w2.get()

        # Update status labels
        ox, oy, oz = node.body_omega
        lbl_omega.config(
            text=f'body \u03c9: [{ox:+.4f}, {oy:+.4f}, {oz:+.4f}] rad/s')

        g1_deg = math.degrees(node.gimbal_pos[0])
        g2_deg = math.degrees(node.gimbal_pos[1])
        lbl_gamma.config(
            text=f'gimbal:  \u03b31={g1_deg:+6.1f}\u00b0  '
                 f'\u03b32={g2_deg:+6.1f}\u00b0')

        rpm1 = node.wheel_vel_rads[0] * 60.0 / (2.0 * math.pi)
        rpm2 = node.wheel_vel_rads[1] * 60.0 / (2.0 * math.pi)
        lbl_rpm.config(
            text=f'wheel:   \u03d61={rpm1:+7.0f}  '
                 f'\u03d62={rpm2:+7.0f} RPM')

        root.after(100, update)

    root.after(500, update)
    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CMGTestbedNode()

    # Spin ROS 2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run tkinter GUI in main thread
    try:
        create_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
