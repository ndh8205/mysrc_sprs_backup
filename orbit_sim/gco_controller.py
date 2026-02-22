import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Vector3
import math
import numpy as np

class PIDController:
    """간단한 PID 제어기 클래스"""
    def __init__(self, p, i, d, max_output):
        self.p_gain = p
        self.i_gain = i
        self.d_gain = d
        self.max_output = max_output
        self.integral = np.zeros(3)
        self.prev_error = np.zeros(3)

    def update(self, error, dt):
        if dt == 0:
            return np.zeros(3)
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        output = (self.p_gain * error + 
                  self.i_gain * self.integral + 
                  self.d_gain * derivative)
        
        return np.clip(output, -self.max_output, self.max_output)

class GCOController(Node):
    def __init__(self):
        super().__init__('gco_controller')

        # ### 수정된 부분: 월드 파일의 위성 이름과 일치시킴 ###
        self.n = 0.001132
        self.radius = 12.0
        self.center_sat_name = 'nasa_satellite2' # 중심 위성
        self.deputy_sat_names = {
            'nasa_satellite3': 0.0,   # 궤도 위성 1 (위상 0도)
            'nasa_satellite4': 90.0   # 궤도 위성 2 (위상 90도)
        }
        # ######################################################

        self.current_poses = {}
        self.last_update_time = None
        self.pids = {name: PIDController(p=5.0, i=0.1, d=2.5, max_output=10.0) 
                     for name in self.deputy_sat_names}
        self.wrench_publishers = {}

        for name in self.deputy_sat_names:
            self.create_subscription(
                Odometry,
                f'/model/{name}/odometry',
                lambda msg, sat_name=name: self.odometry_callback(msg, sat_name),
                10)
            self.wrench_publishers[name] = self.create_publisher(
                Wrench,
                f'/model/{name}/apply_world_wrench',
                10)

        timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(timer_period, self.control_loop)
        self.get_logger().info('GCO Controller Node has been started for nasa_satellite models.')

    def odometry_callback(self, msg, satellite_name):
        self.current_poses[satellite_name] = msg.pose.pose

    def get_target_pos(self, t, phase_deg):
        phase_rad = math.radians(phase_deg)
        r_0 = (self.radius / 2.0) * math.sin(phase_rad)
        v_0 = (self.radius * self.n / 2.0) * math.cos(phase_rad)
        nt = self.n * t
        x = (v_0 / self.n) * math.sin(nt) + r_0 * math.cos(nt)
        y = (2.0 * v_0 / self.n) * math.cos(nt) - 2.0 * r_0 * math.sin(nt)
        z = math.sqrt(3.0) * x
        return np.array([x, y, z])

    def control_loop(self):
        current_time = self.get_clock().now()
        if self.last_update_time is None:
            self.last_update_time = current_time
            return
        if len(self.current_poses) < len(self.deputy_sat_names):
            self.get_logger().warn('Waiting for all satellite poses...', throttle_duration_sec=5)
            return

        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        sim_time = current_time.nanoseconds / 1e9

        for name, phase in self.deputy_sat_names.items():
            target_pos = self.get_target_pos(sim_time, phase)
            current_pose = self.current_poses[name]
            current_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
            error = target_pos - current_pos
            force_vector = self.pids[name].update(error, dt)
            wrench_msg = Wrench(force=Vector3(x=force_vector[0], y=force_vector[1], z=force_vector[2]))
            self.wrench_publishers[name].publish(wrench_msg)
            
        self.last_update_time = current_time

def main(args=None):
    rclpy.init(args=args)
    gco_controller = GCOController()
    rclpy.spin(gco_controller)
    gco_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()