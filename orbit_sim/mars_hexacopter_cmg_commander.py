"""
Mars Hexacopter CMG Commander GUI
- Zero-gravity CMG-only 테스트용
- tkinter 슬라이더로 짐벌 각도(deg) + 휠 RPM 제어
- Sync/Opposite 체크박스
- 짐벌: 속도제어 + 소프트웨어 위치루프
- 휠: 직접 속도 명령 (RPM → rad/s)
- cmg_testbed_commander.py 기반, 토픽만 mars_hexacopter로 변경
"""

import threading
import math
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, JointState


GAMMA_DOT_MAX = 1.0  # rad/s


class MarsHexCMGNode(Node):
    def __init__(self):
        super().__init__('mars_hexacopter_cmg_commander')

        self.declare_parameter('model_name', 'mars_hexacopter')
        self.declare_parameter('max_rpm', 20000)
        self.model_name = self.get_parameter('model_name').value
        self.max_rpm = self.get_parameter('max_rpm').value

        # World name convention: <model>_world
        world_name = f'{self.model_name}_world'

        self.pub_gimbal = [
            self.create_publisher(Float64, f'/{self.model_name}/cmg1_gimbal/cmd_vel', 10),
            self.create_publisher(Float64, f'/{self.model_name}/cmg2_gimbal/cmd_vel', 10),
        ]
        self.pub_wheel = [
            self.create_publisher(Float64, f'/{self.model_name}/cmg1_wheel/cmd_vel', 10),
            self.create_publisher(Float64, f'/{self.model_name}/cmg2_wheel/cmd_vel', 10),
        ]

        self.create_subscription(
            JointState, f'/{self.model_name}/joint_states', self.joint_cb, 10)
        self.create_subscription(
            Imu,
            f'/world/{world_name}/model/{self.model_name}/link/base_link/sensor/imu_sensor/imu',
            self.imu_cb, 10)

        self.gimbal_pos = [0.0, 0.0]
        self.wheel_vel_rads = [0.0, 0.0]
        self.body_omega = [0.0, 0.0, 0.0]
        self.body_quat = [0.0, 0.0, 0.0, 1.0]  # x,y,z,w

        self.cmd_gimbal_deg = [0.0, 0.0]
        self.cmd_rpm = [0.0, 0.0]

        self.gimbal_kp = 5.0

        self.create_timer(0.01, self.publish_commands)
        self.get_logger().info(
            f'Mars Hexacopter CMG Commander started '
            f'(model={self.model_name}, max_rpm={self.max_rpm})')

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
        q = msg.orientation
        self.body_quat = [q.x, q.y, q.z, q.w]

    def publish_commands(self):
        for i in range(2):
            target_rad = math.radians(self.cmd_gimbal_deg[i])
            error = target_rad - self.gimbal_pos[i]
            vel_cmd = max(-GAMMA_DOT_MAX, min(GAMMA_DOT_MAX, self.gimbal_kp * error))
            msg = Float64()
            msg.data = vel_cmd
            self.pub_gimbal[i].publish(msg)

        for i in range(2):
            msg = Float64()
            msg.data = self.cmd_rpm[i] * (2.0 * math.pi / 60.0)
            self.pub_wheel[i].publish(msg)


def quat_to_euler(x, y, z, w):
    """Quaternion (x,y,z,w) → Euler (roll, pitch, yaw) in degrees."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def create_gui(node):
    BG = '#252530'
    FG = '#ffffff'
    FG_DIM = '#999999'
    CMG1_COLOR = '#ff6633'
    CMG2_COLOR = '#3399ff'

    max_rpm = node.max_rpm

    root = tk.Tk()
    root.title(f'CMG Commander — {node.model_name}')
    root.configure(bg=BG)
    root.geometry('540x700')
    root.resizable(False, False)

    # --- Attitude section ---
    frm_att = tk.LabelFrame(
        root, text='Attitude', bg=BG, fg='#66ff66',
        font=('Consolas', 10, 'bold'))
    frm_att.pack(fill='x', padx=8, pady=(8, 4))

    lbl_euler = tk.Label(
        frm_att, text='RPY: ---', bg=BG, fg=FG,
        font=('Consolas', 10), anchor='w')
    lbl_euler.pack(fill='x', padx=6, pady=2)

    lbl_omega = tk.Label(
        frm_att, text='omega: ---', bg=BG, fg=FG,
        font=('Consolas', 10), anchor='w')
    lbl_omega.pack(fill='x', padx=6, pady=2)

    lbl_gamma = tk.Label(
        frm_att, text='gimbal: ---', bg=BG, fg=FG,
        font=('Consolas', 10), anchor='w')
    lbl_gamma.pack(fill='x', padx=6, pady=2)

    lbl_rpm = tk.Label(
        frm_att, text='wheel:  ---', bg=BG, fg=FG,
        font=('Consolas', 10), anchor='w')
    lbl_rpm.pack(fill='x', padx=6, pady=(2, 6))

    # --- Gimbal section ---
    frm_gimbal = tk.LabelFrame(
        root, text='Gimbal angle [deg]', bg=BG, fg=CMG1_COLOR,
        font=('Consolas', 10, 'bold'))
    frm_gimbal.pack(fill='x', padx=8, pady=4)

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

    frm_g1 = tk.Frame(frm_gimbal, bg=BG)
    frm_g1.pack(fill='x', padx=6, pady=2)
    tk.Label(frm_g1, text='\u03b31:', bg=BG, fg=CMG1_COLOR,
             font=('Consolas', 10, 'bold'), width=3).pack(side='left')
    var_g1 = tk.DoubleVar(value=0.0)
    tk.Scale(
        frm_g1, from_=-90, to=90, orient='horizontal', variable=var_g1,
        bg='#333340', fg=CMG1_COLOR, troughcolor='#1a1a24',
        highlightthickness=0, font=('Consolas', 8), length=340,
        resolution=0.1, showvalue=False).pack(side='left', padx=4)
    tk.Entry(
        frm_g1, textvariable=var_g1, bg='#333340', fg=CMG1_COLOR,
        font=('Consolas', 10), width=7, justify='center').pack(side='left', padx=4)

    frm_g2 = tk.Frame(frm_gimbal, bg=BG)
    frm_g2.pack(fill='x', padx=6, pady=(2, 6))
    tk.Label(frm_g2, text='\u03b32:', bg=BG, fg=CMG2_COLOR,
             font=('Consolas', 10, 'bold'), width=3).pack(side='left')
    var_g2 = tk.DoubleVar(value=0.0)
    tk.Scale(
        frm_g2, from_=-90, to=90, orient='horizontal', variable=var_g2,
        bg='#333340', fg=CMG2_COLOR, troughcolor='#1a1a24',
        highlightthickness=0, font=('Consolas', 8), length=340,
        resolution=0.1, showvalue=False).pack(side='left', padx=4)
    tk.Entry(
        frm_g2, textvariable=var_g2, bg='#333340', fg=CMG2_COLOR,
        font=('Consolas', 10), width=7, justify='center').pack(side='left', padx=4)

    # --- Wheel section ---
    frm_wheel = tk.LabelFrame(
        root, text='Wheel speed [RPM]', bg=BG, fg=CMG2_COLOR,
        font=('Consolas', 10, 'bold'))
    frm_wheel.pack(fill='x', padx=8, pady=4)

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

    frm_w1 = tk.Frame(frm_wheel, bg=BG)
    frm_w1.pack(fill='x', padx=6, pady=2)
    tk.Label(frm_w1, text='\u03d61:', bg=BG, fg=CMG1_COLOR,
             font=('Consolas', 10, 'bold'), width=3).pack(side='left')
    var_w1 = tk.DoubleVar(value=0.0)
    tk.Scale(
        frm_w1, from_=-max_rpm, to=max_rpm, orient='horizontal', variable=var_w1,
        bg='#333340', fg=CMG1_COLOR, troughcolor='#1a1a24',
        highlightthickness=0, font=('Consolas', 8), length=340,
        resolution=100, showvalue=False).pack(side='left', padx=4)
    tk.Entry(
        frm_w1, textvariable=var_w1, bg='#333340', fg=CMG1_COLOR,
        font=('Consolas', 10), width=7, justify='center').pack(side='left', padx=4)

    frm_w2 = tk.Frame(frm_wheel, bg=BG)
    frm_w2.pack(fill='x', padx=6, pady=(2, 6))
    tk.Label(frm_w2, text='\u03d62:', bg=BG, fg=CMG2_COLOR,
             font=('Consolas', 10, 'bold'), width=3).pack(side='left')
    var_w2 = tk.DoubleVar(value=0.0)
    tk.Scale(
        frm_w2, from_=-max_rpm, to=max_rpm, orient='horizontal', variable=var_w2,
        bg='#333340', fg=CMG2_COLOR, troughcolor='#1a1a24',
        highlightthickness=0, font=('Consolas', 8), length=340,
        resolution=100, showvalue=False).pack(side='left', padx=4)
    tk.Entry(
        frm_w2, textvariable=var_w2, bg='#333340', fg=CMG2_COLOR,
        font=('Consolas', 10), width=7, justify='center').pack(side='left', padx=4)

    # --- Buttons ---
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
        activebackground='#555565', activeforeground=FG, width=12
    ).pack(side='left', padx=4)

    tk.Label(
        frm_btn, text=f'{node.model_name} | DART physics',
        bg=BG, fg=FG_DIM, font=('Consolas', 8)
    ).pack(side='right', padx=4)

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

    def update():
        node.cmd_gimbal_deg[0] = var_g1.get()
        node.cmd_gimbal_deg[1] = var_g2.get()
        node.cmd_rpm[0] = var_w1.get()
        node.cmd_rpm[1] = var_w2.get()

        # Attitude
        r, p, y = quat_to_euler(*node.body_quat)
        lbl_euler.config(text=f'RPY: [{r:+7.2f}, {p:+7.2f}, {y:+7.2f}] deg')

        ox, oy, oz = node.body_omega
        lbl_omega.config(
            text=f'\u03c9:   [{ox:+.4f}, {oy:+.4f}, {oz:+.4f}] rad/s')

        g1d = math.degrees(node.gimbal_pos[0])
        g2d = math.degrees(node.gimbal_pos[1])
        lbl_gamma.config(
            text=f'gimbal: \u03b31={g1d:+6.1f}\u00b0  \u03b32={g2d:+6.1f}\u00b0')

        rpm1 = node.wheel_vel_rads[0] * 60.0 / (2.0 * math.pi)
        rpm2 = node.wheel_vel_rads[1] * 60.0 / (2.0 * math.pi)
        lbl_rpm.config(
            text=f'wheel:  \u03d61={rpm1:+7.0f}  \u03d62={rpm2:+7.0f} RPM')

        root.after(100, update)

    root.after(500, update)
    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = MarsHexCMGNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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
