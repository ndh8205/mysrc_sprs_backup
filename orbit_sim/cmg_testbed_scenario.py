"""
CMG Testbed — Automated test scenario + CSV data logger
- No attitude control: direct gimbal angle + wheel RPM profile
- Logs IMU + joint_states to CSV for MATLAB comparison
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64

import numpy as np
import csv
import os
import math
from datetime import datetime


# ============================================================
# Scenario waypoints (linear interpolation between points)
# (time_s, gimbal1_deg, gimbal2_deg, wheel1_rpm, wheel2_rpm)
# ============================================================
WAYPOINTS = [
    #  t[s]   g1[deg]  g2[deg]  w1[RPM]    w2[RPM]
    (  0.0,    0.0,     0.0,       0.0,       0.0),
    (  5.0,    0.0,     0.0,   10000.0,  -10000.0),   # wheel spin-up
    ( 10.0,   30.0,     0.0,   10000.0,  -10000.0),   # sweep gimbal1 → 30deg
    ( 20.0,   30.0,     0.0,   10000.0,  -10000.0),   # hold
    ( 25.0,    0.0,     0.0,   10000.0,  -10000.0),   # return gimbal1 → 0deg
    ( 30.0,    0.0,     0.0,   10000.0,  -10000.0),   # hold & observe
]

SCENARIO_DURATION = WAYPOINTS[-1][0]

# Gimbal angle tracking (Gazebo JointController = velocity cmd only)
GIMBAL_KP = 5.0
GIMBAL_VEL_MAX = 1.0  # rad/s


def quat_to_euler_deg(w, x, y, z):
    """Quaternion (w,x,y,z) -> [roll, pitch, yaw] degrees"""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def interpolate(t, waypoints):
    """Linear interpolation of waypoint values at time t"""
    if t <= waypoints[0][0]:
        return waypoints[0][1:]
    if t >= waypoints[-1][0]:
        return waypoints[-1][1:]

    for i in range(len(waypoints) - 1):
        t0 = waypoints[i][0]
        t1 = waypoints[i + 1][0]
        if t0 <= t < t1:
            a = (t - t0) / (t1 - t0)
            v0 = np.array(waypoints[i][1:])
            v1 = np.array(waypoints[i + 1][1:])
            return tuple(v0 + a * (v1 - v0))

    return waypoints[-1][1:]


# ============================================================
# ROS 2 Node
# ============================================================

class CMGTestbedScenario(Node):
    def __init__(self):
        super().__init__('cmg_testbed_scenario')

        # --- State ---
        self.imu_q = [1.0, 0.0, 0.0, 0.0]   # w, x, y, z
        self.imu_omega = [0.0, 0.0, 0.0]
        self.joint_pos = {}
        self.joint_vel = {}
        self.imu_ok = False
        self.joint_ok = False

        self.start_time = None
        self.finished = False
        self.data = []
        self.last_log_time = -999.0

        # --- Subscribers ---
        self.create_subscription(Imu, '/cmg_testbed/imu', self.imu_cb, 10)
        self.create_subscription(
            JointState, '/cmg_testbed/joint_states', self.joint_cb, 10)

        # --- Publishers ---
        self.pub_g1 = self.create_publisher(
            Float64, '/cmg_testbed/cmg1_gimbal/cmd_vel', 10)
        self.pub_g2 = self.create_publisher(
            Float64, '/cmg_testbed/cmg2_gimbal/cmd_vel', 10)
        self.pub_w1 = self.create_publisher(
            Float64, '/cmg_testbed/cmg1_wheel/cmd_vel', 10)
        self.pub_w2 = self.create_publisher(
            Float64, '/cmg_testbed/cmg2_wheel/cmd_vel', 10)

        # --- Timer 100 Hz ---
        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info(
            f'CMG Testbed Scenario (duration={SCENARIO_DURATION}s)')
        self.get_logger().info('Waiting for IMU + joint_states ...')

    # ---- Callbacks ----

    def imu_cb(self, msg):
        o = msg.orientation
        self.imu_q = [o.w, o.x, o.y, o.z]
        av = msg.angular_velocity
        self.imu_omega = [av.x, av.y, av.z]
        self.imu_ok = True

    def joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_pos[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_vel[name] = msg.velocity[i]
        self.joint_ok = True

    # ---- Main loop ----

    def control_loop(self):
        if not (self.imu_ok and self.joint_ok):
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info('Scenario started!')

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if t > SCENARIO_DURATION:
            if not self.finished:
                self.finished = True
                self.pub_g1.publish(Float64(data=0.0))
                self.pub_g2.publish(Float64(data=0.0))
                self.pub_w1.publish(Float64(data=0.0))
                self.pub_w2.publish(Float64(data=0.0))
                self.save_csv()
                self.get_logger().info('Scenario complete!')
                self.timer.cancel()
            return

        # Interpolate profile
        g1_deg, g2_deg, w1_rpm, w2_rpm = interpolate(t, WAYPOINTS)

        # Gimbal: angle command → velocity (Gazebo only accepts vel cmd)
        g1_pos = self.joint_pos.get('cmg1_gimbal_joint', 0.0)
        g2_pos = self.joint_pos.get('cmg2_gimbal_joint', 0.0)
        g1_vel = max(-GIMBAL_VEL_MAX, min(GIMBAL_VEL_MAX,
                     GIMBAL_KP * (math.radians(g1_deg) - g1_pos)))
        g2_vel = max(-GIMBAL_VEL_MAX, min(GIMBAL_VEL_MAX,
                     GIMBAL_KP * (math.radians(g2_deg) - g2_pos)))

        self.pub_g1.publish(Float64(data=g1_vel))
        self.pub_g2.publish(Float64(data=g2_vel))

        # Wheel: RPM → rad/s
        self.pub_w1.publish(Float64(data=w1_rpm * 2.0 * math.pi / 60.0))
        self.pub_w2.publish(Float64(data=w2_rpm * 2.0 * math.pi / 60.0))

        # ---- Record data ----
        qw, qx, qy, qz = self.imu_q
        wx, wy, wz = self.imu_omega
        roll, pitch, yaw = quat_to_euler_deg(qw, qx, qy, qz)

        gamma1 = self.joint_pos.get('cmg1_gimbal_joint', 0.0)
        gamma2 = self.joint_pos.get('cmg2_gimbal_joint', 0.0)
        varpi1 = self.joint_vel.get('cmg1_wheel_joint', 0.0)
        varpi2 = self.joint_vel.get('cmg2_wheel_joint', 0.0)

        self.data.append([
            t,
            qw, qx, qy, qz,
            wx, wy, wz,
            math.degrees(gamma1), math.degrees(gamma2),
            varpi1 * 60.0 / (2.0 * math.pi),
            varpi2 * 60.0 / (2.0 * math.pi),
            roll, pitch, yaw,
        ])

        # Console log every 5s
        if t - self.last_log_time >= 5.0:
            self.last_log_time = t
            self.get_logger().info(
                f't={t:5.1f}s  euler=[{roll:+7.2f},{pitch:+7.2f},{yaw:+7.2f}]  '
                f'gamma=[{math.degrees(gamma1):+6.1f},{math.degrees(gamma2):+6.1f}]  '
                f'wheel=[{varpi1*60/(2*math.pi):+7.0f},'
                f'{varpi2*60/(2*math.pi):+7.0f}] RPM')

    # ---- CSV output ----

    def save_csv(self):
        data_dir = '/home/ndh/space_ros_ws/src/orbit_sim/data'
        os.makedirs(data_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(data_dir, f'cmg_testbed_{ts}.csv')

        header = [
            'time',
            'q_w', 'q_x', 'q_y', 'q_z',
            'omega_x', 'omega_y', 'omega_z',
            'gamma1_deg', 'gamma2_deg',
            'varpi1_rpm', 'varpi2_rpm',
            'euler_roll_deg', 'euler_pitch_deg', 'euler_yaw_deg',
        ]

        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.data)

        self.get_logger().info(
            f'Saved {len(self.data)} samples -> {filepath}')


def main(args=None):
    rclpy.init(args=args)
    node = CMGTestbedScenario()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
