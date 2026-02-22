#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import threading
from scipy.spatial.transform import Rotation

class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        
        # 파라미터 선언
        self.declare_parameter('satellite_name', 'nasa_satellite2')
        self.declare_parameter('history_length', 10)  # 표시할 데이터 포인트 수
        self.declare_parameter('update_rate', 1.0)    # Hz
        self.declare_parameter('quaternion_order', 'xyzw')  # 'xyzw' or 'wxyz'
        self.declare_parameter('quaternion_convention', 'hamilton')  # 'hamilton' or 'jpl'
        
        # 파라미터 가져오기
        self.satellite_name = self.get_parameter('satellite_name').value
        self.history_length = self.get_parameter('history_length').value
        self.update_rate = self.get_parameter('update_rate').value
        self.quaternion_order = self.get_parameter('quaternion_order').value
        self.quaternion_convention = self.get_parameter('quaternion_convention').value
        
        # IMU 토픽 구독
        self.subscription = self.create_subscription(
            Imu,
            f'/{self.satellite_name}/imu',
            self.imu_callback,
            10
        )
        
        # 데이터 저장용 deque
        self.timestamps = deque(maxlen=self.history_length)
        self.euler_angles = {
            'roll': deque(maxlen=self.history_length),
            'pitch': deque(maxlen=self.history_length),
            'yaw': deque(maxlen=self.history_length)
        }
        self.angular_velocity = {
            'x': deque(maxlen=self.history_length),
            'y': deque(maxlen=self.history_length),
            'z': deque(maxlen=self.history_length)
        }
        self.linear_acceleration = {
            'x': deque(maxlen=self.history_length),
            'y': deque(maxlen=self.history_length),
            'z': deque(maxlen=self.history_length)
        }
        
        # 현재 쿼터니언 저장
        self.current_quaternion = [0, 0, 0, 1]  # x, y, z, w
        
        # 시작 시간
        self.start_time = self.get_clock().now()
        
        # 플롯 설정
        self.setup_plots()
        
        # 데이터 업데이트 플래그
        self.data_updated = False
        self.lock = threading.Lock()
        
        # 첫 메시지 수신 플래그
        self.first_message_received = False
        
        self.get_logger().info(f'IMU Visualizer started for {self.satellite_name}')
        self.get_logger().info(f'Quaternion order: {self.quaternion_order}, Convention: {self.quaternion_convention}')
    
    def imu_callback(self, msg):
        """IMU 메시지 콜백"""
        with self.lock:
            # 타임스탬프 계산
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            self.timestamps.append(elapsed_time)
            
            # 쿼터니언 추출 및 순서 처리
            q = msg.orientation
            
            # ROS 메시지는 x,y,z,w 순서로 저장됨
            if self.quaternion_order == 'xyzw':
                quat_array = [q.x, q.y, q.z, q.w]
            else:  # wxyz
                quat_array = [q.w, q.x, q.y, q.z]
                # scipy는 x,y,z,w 순서를 기대하므로 변환
                quat_array = [quat_array[1], quat_array[2], quat_array[3], quat_array[0]]
            
            # JPL 컨벤션인 경우, 해밀토니언으로 변환 (공액 취함)
            if self.quaternion_convention == 'jpl':
                quat_array[0] = -quat_array[0]  # x
                quat_array[1] = -quat_array[1]  # y
                quat_array[2] = -quat_array[2]  # z
                # w는 그대로
            
            self.current_quaternion = quat_array
            
            # 첫 메시지에서 쿼터니언 검증
            if not self.first_message_received:
                self.verify_quaternion(quat_array)
                self.first_message_received = True
            
            # scipy를 사용한 변환
            r = Rotation.from_quat(quat_array)
            euler = r.as_euler('xyz', degrees=True)  # roll, pitch, yaw in degrees
            
            self.euler_angles['roll'].append(euler[0])
            self.euler_angles['pitch'].append(euler[1])
            self.euler_angles['yaw'].append(euler[2])
            
            # 각속도 저장
            self.angular_velocity['x'].append(msg.angular_velocity.x)
            self.angular_velocity['y'].append(msg.angular_velocity.y)
            self.angular_velocity['z'].append(msg.angular_velocity.z)
            
            # 선형 가속도 저장
            self.linear_acceleration['x'].append(msg.linear_acceleration.x)
            self.linear_acceleration['y'].append(msg.linear_acceleration.y)
            self.linear_acceleration['z'].append(msg.linear_acceleration.z)
            
            self.data_updated = True
    
    def verify_quaternion(self, quat):
        """쿼터니언 유효성 검증 및 정보 출력"""
        norm = np.sqrt(sum([q**2 for q in quat]))
        self.get_logger().info(f'First quaternion received: x={quat[0]:.4f}, y={quat[1]:.4f}, z={quat[2]:.4f}, w={quat[3]:.4f}')
        self.get_logger().info(f'Quaternion norm: {norm:.6f} (should be close to 1.0)')
        
        if abs(norm - 1.0) > 0.01:
            self.get_logger().warn(f'Quaternion norm is {norm:.6f}, which is not close to 1.0!')
            self.get_logger().warn('Check quaternion order and convention parameters.')
    
    def setup_plots(self):
        """플롯 초기 설정"""
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(15, 10))
        self.fig.suptitle(f'IMU Data Visualization - {self.satellite_name}', fontsize=16)
        
        # 서브플롯 생성
        # 1. 오일러 각도
        self.ax_euler = self.fig.add_subplot(2, 2, 1)
        self.ax_euler.set_title('Euler Angles')
        self.ax_euler.set_xlabel('Time (s)')
        self.ax_euler.set_ylabel('Angle (degrees)')
        self.ax_euler.grid(True, alpha=0.3)
        
        # 2. 각속도
        self.ax_angular = self.fig.add_subplot(2, 2, 2)
        self.ax_angular.set_title('Angular Velocity')
        self.ax_angular.set_xlabel('Time (s)')
        self.ax_angular.set_ylabel('Angular Velocity (rad/s)')
        self.ax_angular.grid(True, alpha=0.3)
        
        # 3. 선형 가속도
        self.ax_linear = self.fig.add_subplot(2, 2, 3)
        self.ax_linear.set_title('Linear Acceleration')
        self.ax_linear.set_xlabel('Time (s)')
        self.ax_linear.set_ylabel('Acceleration (m/s²)')
        self.ax_linear.grid(True, alpha=0.3)
        
        # 4. 3D 자세 시각화
        self.ax_3d = self.fig.add_subplot(2, 2, 4, projection='3d')
        self.ax_3d.set_title('3D Attitude Visualization')
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_xlim([-1.5, 1.5])
        self.ax_3d.set_ylim([-1.5, 1.5])
        self.ax_3d.set_zlim([-1.5, 1.5])
        
        # 라인 객체 초기화
        self.lines_euler = {
            'roll': self.ax_euler.plot([], [], 'r-', label='Roll')[0],
            'pitch': self.ax_euler.plot([], [], 'g-', label='Pitch')[0],
            'yaw': self.ax_euler.plot([], [], 'b-', label='Yaw')[0]
        }
        self.ax_euler.legend()
        
        self.lines_angular = {
            'x': self.ax_angular.plot([], [], 'r-', label='X')[0],
            'y': self.ax_angular.plot([], [], 'g-', label='Y')[0],
            'z': self.ax_angular.plot([], [], 'b-', label='Z')[0]
        }
        self.ax_angular.legend()
        
        self.lines_linear = {
            'x': self.ax_linear.plot([], [], 'r-', label='X')[0],
            'y': self.ax_linear.plot([], [], 'g-', label='Y')[0],
            'z': self.ax_linear.plot([], [], 'b-', label='Z')[0]
        }
        self.ax_linear.legend()
        
        # 3D 축 초기화
        self.axis_lines = None
        
        plt.tight_layout()
    
    def update_plots(self, frame):
        """플롯 업데이트"""
        with self.lock:
            if not self.data_updated or len(self.timestamps) < 2:
                return
            
            # 시간 데이터
            times = list(self.timestamps)
            
            # 1. 오일러 각도 업데이트
            for key in ['roll', 'pitch', 'yaw']:
                self.lines_euler[key].set_data(times, list(self.euler_angles[key]))
            
            # 2. 각속도 업데이트
            for key in ['x', 'y', 'z']:
                self.lines_angular[key].set_data(times, list(self.angular_velocity[key]))
            
            # 3. 선형 가속도 업데이트
            for key in ['x', 'y', 'z']:
                self.lines_linear[key].set_data(times, list(self.linear_acceleration[key]))
            
            # 축 범위 자동 조정
            if times:
                time_margin = 0.1 * (times[-1] - times[0]) if times[-1] != times[0] else 1
                
                # 오일러 각도
                self.ax_euler.set_xlim(times[0] - time_margin, times[-1] + time_margin)
                all_euler = [val for angles in self.euler_angles.values() for val in angles]
                if all_euler:
                    self.ax_euler.set_ylim(min(all_euler) - 10, max(all_euler) + 10)
                
                # 각속도
                self.ax_angular.set_xlim(times[0] - time_margin, times[-1] + time_margin)
                all_angular = [val for vel in self.angular_velocity.values() for val in vel]
                if all_angular:
                    margin = 0.1 * (max(all_angular) - min(all_angular))
                    self.ax_angular.set_ylim(min(all_angular) - margin, max(all_angular) + margin)
                
                # 선형 가속도
                self.ax_linear.set_xlim(times[0] - time_margin, times[-1] + time_margin)
                all_linear = [val for acc in self.linear_acceleration.values() for val in acc]
                if all_linear:
                    margin = 0.1 * (max(all_linear) - min(all_linear))
                    self.ax_linear.set_ylim(min(all_linear) - margin, max(all_linear) + margin)
            
            # 4. 3D 자세 업데이트
            self.update_3d_attitude()
            
            self.data_updated = False
    
    def update_3d_attitude(self):
        """3D 자세 시각화 업데이트"""
        # 이전 축 제거
        if self.axis_lines:
            for line in self.axis_lines:
                line.remove()
        
        # 쿼터니언을 회전 행렬로 변환
        r = Rotation.from_quat(self.current_quaternion)
        rotation_matrix = r.as_matrix()
        
        # 축 벡터
        axis_length = 1.0
        axes = np.array([
            [axis_length, 0, 0],  # X축 (빨강)
            [0, axis_length, 0],  # Y축 (초록)
            [0, 0, axis_length]   # Z축 (파랑)
        ])
        
        # 회전 적용
        rotated_axes = np.dot(rotation_matrix, axes.T).T
        
        # 축 그리기
        self.axis_lines = []
        colors = ['r', 'g', 'b']
        labels = ['X', 'Y', 'Z']
        
        for i, (color, label) in enumerate(zip(colors, labels)):
            line = self.ax_3d.plot([0, rotated_axes[i, 0]], 
                                  [0, rotated_axes[i, 1]], 
                                  [0, rotated_axes[i, 2]], 
                                  color=color, linewidth=3, label=label)[0]
            self.axis_lines.append(line)
            
            # 축 끝에 라벨 추가
            self.ax_3d.text(rotated_axes[i, 0] * 1.1, 
                           rotated_axes[i, 1] * 1.1, 
                           rotated_axes[i, 2] * 1.1, 
                           label, color=color, fontsize=12)
        
        # 원점에 점 표시
        self.ax_3d.scatter([0], [0], [0], color='white', s=50)
    
    def start_visualization(self):
        """시각화 시작"""
        # 애니메이션 설정
        self.ani = FuncAnimation(self.fig, self.update_plots, 
                               interval=int(1000 / self.update_rate), 
                               blit=False)
        
        # 플롯 표시
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    
    node = IMUVisualizer()
    
    # 별도 스레드에서 ROS 실행
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node))
    ros_thread.start()
    
    # 메인 스레드에서 시각화 실행
    try:
        node.start_visualization()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()