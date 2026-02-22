#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import csv
import numpy as np
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose, Twist
import math
import threading
import os

class MultiSatelliteController(Node):
    def __init__(self):
        super().__init__('multi_satellite_controller')
        
        # 개별 CSV 파일 경로, update_rate, loop_data 파라미터 선언
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('loop_data', True)
        self.declare_parameter('csv_file1', '')
        self.declare_parameter('csv_file2', '')
        self.declare_parameter('csv_file3', '')
        
        # 파라미터 값 읽기
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.loop_data = self.get_parameter('loop_data').get_parameter_value().bool_value
        csv_file1 = self.get_parameter('csv_file1').get_parameter_value().string_value
        csv_file2 = self.get_parameter('csv_file2').get_parameter_value().string_value
        csv_file3 = self.get_parameter('csv_file3').get_parameter_value().string_value
        
        # 위성 이름과 CSV 파일 경로 매핑 구성
        self.satellites = {}
        if csv_file1:
            self.satellites["nasa_satellite"] = {
                'csv_path': os.path.expanduser(csv_file1),
                'data': {
                    'time_index': [],
                    'x': [], 'vx': [],
                    'y': [], 'vy': [],
                    'z': [], 'vz': []
                },
                'current_index': 0
            }
            self.get_logger().info(f'Added satellite nasa_satellite with CSV file: {csv_file1}')
        if csv_file2:
            self.satellites["nasa_satellite2"] = {
                'csv_path': os.path.expanduser(csv_file2),
                'data': {
                    'time_index': [],
                    'x': [], 'vx': [],
                    'y': [], 'vy': [],
                    'z': [], 'vz': []
                },
                'current_index': 0
            }
            self.get_logger().info(f'Added satellite nasa_satellite2 with CSV file: {csv_file2}')
        if csv_file3:
            self.satellites["nasa_satellite3"] = {
                'csv_path': os.path.expanduser(csv_file3),
                'data': {
                    'time_index': [],
                    'x': [], 'vx': [],
                    'y': [], 'vy': [],
                    'z': [], 'vz': []
                },
                'current_index': 0
            }
            self.get_logger().info(f'Added satellite nasa_satellite3 with CSV file: {csv_file3}')
        
        if not self.satellites:
            self.get_logger().error('No satellite CSV mappings provided!')
            return
        
        # 타이머 생성: 주기적으로 위성 위치 업데이트
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_satellite_positions)
        
        # 엔티티 제어 퍼블리셔 (ROS: Entity -> Gazebo: EntityCmd)
        self.entity_pub = self.create_publisher(
            Entity,
            '/world/space_world/set_pose',
            QoSProfile(depth=10)
        )
        
        # CSV 데이터 로드를 위한 스레드 시작
        self.load_thread = threading.Thread(target=self.load_all_csv_data)
        self.load_thread.daemon = True
        self.load_thread.start()
        
        self.is_running = True
        self.data_loaded = False
        
        self.get_logger().info(f'Multi-satellite controller initialized for {len(self.satellites)} satellites')
    
    def load_all_csv_data(self):
        """모든 위성의 CSV 데이터를 로드 (별도 스레드)"""
        for sat_name, sat_info in self.satellites.items():
            self.get_logger().info(f'Loading CSV data for {sat_name} from {sat_info["csv_path"]}')
            self.load_csv_data(sat_name, sat_info)
        self.data_loaded = True
        self.get_logger().info('All CSV data loaded successfully')
    
    def load_csv_data(self, sat_name, sat_info):
        """특정 위성의 CSV 데이터를 로드"""
        if not os.path.exists(sat_info['csv_path']):
            self.get_logger().error(f'CSV file not found: {sat_info["csv_path"]}')
            return
        try:
            with open(sat_info['csv_path'], 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                fieldnames = reader.fieldnames
                time_index_field = next((f for f in fieldnames if 'Time_Index' in f), None)
                x_field = next((f for f in fieldnames if 'x(m)' in f), None)
                vx_field = next((f for f in fieldnames if 'vx(m/s)' in f), None)
                y_field = next((f for f in fieldnames if 'y(m)' in f), None)
                vy_field = next((f for f in fieldnames if 'vy(m/s)' in f), None)
                z_field = next((f for f in fieldnames if 'z(m)' in f), None)
                vz_field = next((f for f in fieldnames if 'vz(m/s)' in f), None)
                
                if not all([time_index_field, x_field, vx_field, y_field, vy_field, z_field, vz_field]):
                    self.get_logger().error(f'CSV format error for {sat_name}. Available: {fieldnames}')
                    return
                
                row_count = 0
                for row in reader:
                    sat_info['data']['time_index'].append(int(row[time_index_field]))
                    sat_info['data']['x'].append(float(row[x_field]))
                    sat_info['data']['vx'].append(float(row[vx_field]))
                    sat_info['data']['y'].append(float(row[y_field]))
                    sat_info['data']['vy'].append(float(row[vy_field]))
                    sat_info['data']['z'].append(float(row[z_field]))
                    sat_info['data']['vz'].append(float(row[vz_field]))
                    row_count += 1
                    if row_count % 50000 == 0:
                        self.get_logger().info(f'Loaded {row_count} rows for {sat_name}...')
                self.get_logger().info(f'Successfully loaded {row_count} rows for {sat_name}')
        except Exception as e:
            self.get_logger().error(f'Error loading CSV data for {sat_name}: {str(e)}')
    
    def update_satellite_positions(self):
        """주기적으로 위성 위치 업데이트"""
        if not self.is_running or not self.data_loaded:
            return
        
        for sat_name, sat_info in self.satellites.items():
            data = sat_info['data']
            current_index = sat_info['current_index']
            if len(data['time_index']) == 0:
                self.get_logger().warn(f'No CSV data for satellite {sat_name}, skipping update.')
                continue
            
            if current_index >= len(data['time_index']):
                if self.loop_data:
                    sat_info['current_index'] = 0
                    current_index = 0
                    self.get_logger().info(f'Restarting data for {sat_name} from beginning')
                else:
                    continue
            
            # 현재 위치/속도 데이터 추출
            x = data['x'][current_index]
            y = data['y'][current_index]
            z = data['z'][current_index]
            vx = data['vx'][current_index]
            vy = data['vy'][current_index]
            vz = data['vz'][current_index]
            
            # Entity 메시지 생성 및 설정
            msg = Entity()
            msg.name = sat_name
            msg.type = Entity.MODEL
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = float(z)
            msg.twist.linear.x = float(vx)
            msg.twist.linear.y = float(vy)
            msg.twist.linear.z = float(vz)
            
            # 속도 기반 회전 계산 (쿼터니언 계산)
            speed = math.sqrt(vx*vx + vy*vy + vz*vz)
            if speed > 0.001:
                nvx, nvy, nvz = vx/speed, vy/speed, vz/speed
                forward = [nvx, nvy, nvz]
                up = [0.0, 0.0, 1.0]
                if abs(np.dot(forward, up)) > 0.9:
                    up = [0.0, 1.0, 0.0]
                right = np.cross(forward, up)
                right = right / np.linalg.norm(right)
                new_up = np.cross(right, forward)
                new_up = new_up / np.linalg.norm(new_up)
                rotation_matrix = np.column_stack([right, new_up, forward])
                trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
                if trace > 0:
                    s = 0.5 / math.sqrt(trace + 1.0)
                    qw = 0.25 / s
                    qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
                    qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
                    qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
                else:
                    if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                        s = 2.0 * math.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
                        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                        qx = 0.25 * s
                        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                        s = 2.0 * math.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
                        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                        qy = 0.25 * s
                        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                    else:
                        s = 2.0 * math.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
                        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                        qz = 0.25 * s
                msg.pose.orientation.x = qx
                msg.pose.orientation.y = qy
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw
            
            # 메시지 발행 및 인덱스 증가
            self.entity_pub.publish(msg)
            sat_info['current_index'] += 1
            
            if sat_info['current_index'] % 5000 == 0:
                self.get_logger().info(
                    f'{sat_name} updated: index={current_index}, '
                    f'position=({x:.2f}, {y:.2f}, {z:.2f}), '
                    f'velocity=({vx:.2f}, {vy:.2f}, {vz:.2f})'
                )

def main(args=None):
    rclpy.init(args=args)
    controller = MultiSatelliteController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
