#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import threading
import numpy as np
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d

class MultiSatelliteControllerGuidence(Node):
    def __init__(self):
        super().__init__('multi_satellite_controller_guidence')
        
        # 위성 제어 파라미터 선언
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('loop_data', True)
        self.declare_parameter('csv_file1', '')
        self.declare_parameter('csv_file2', '')
        self.declare_parameter('csv_file3', '')
        self.declare_parameter('csv_data_rate', 50.0)
        self.declare_parameter('time_scale', 1.0)
        
        # 파라미터 읽기
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.loop_data = self.get_parameter('loop_data').get_parameter_value().bool_value
        csv_file1 = self.get_parameter('csv_file1').get_parameter_value().string_value
        csv_file2 = self.get_parameter('csv_file2').get_parameter_value().string_value
        csv_file3 = self.get_parameter('csv_file3').get_parameter_value().string_value
        self.csv_data_rate = self.get_parameter('csv_data_rate').get_parameter_value().double_value
        self.time_scale = self.get_parameter('time_scale').get_parameter_value().double_value
        
        # CSV 데이터 간격 계산 (초 단위)
        self.csv_time_step = 1.0 / self.csv_data_rate if self.csv_data_rate > 0 else 0.02
        
        self.get_logger().info(f'CSV 데이터 간격: {self.csv_time_step}초 ({self.csv_data_rate}Hz)')
        self.get_logger().info(f'시간 스케일 팩터: {self.time_scale}x')
        
        # 스레드 안전성을 위한 락
        self.state_lock = threading.Lock()
        
        # 위성 CSV 파일 경로 매핑 구성
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
                'current_time': 0.0,
                'interpolators': {},
                'current_position': np.zeros(3),
                'current_velocity': np.zeros(3),
                'current_orientation': np.array([0.0, 0.0, 0.0, 1.0]),
            }
            self.get_logger().info(f'Added satellite nasa_satellite with CSV file: {csv_file1}')
        if csv_file2:
            self.satellites["nasa_satellite3"] = {
                'csv_path': os.path.expanduser(csv_file2),
                'data': {
                    'time_index': [],
                    'x': [], 'vx': [],
                    'y': [], 'vy': [],
                    'z': [], 'vz': []
                },
                'current_time': 0.0,
                'interpolators': {},
                'current_position': np.zeros(3),
                'current_velocity': np.zeros(3),
                'current_orientation': np.array([0.0, 0.0, 0.0, 1.0]),
            }
            self.get_logger().info(f'Added satellite nasa_satellite2 with CSV file: {csv_file2}')
        if csv_file3:
            self.satellites["nasa_satellite4"] = {
                'csv_path': os.path.expanduser(csv_file3),
                'data': {
                    'time_index': [],
                    'x': [], 'vx': [],
                    'y': [], 'vy': [],
                    'z': [], 'vz': []
                },
                'current_time': 0.0,
                'interpolators': {},
                'current_position': np.zeros(3),
                'current_velocity': np.zeros(3),
                'current_orientation': np.array([0.0, 0.0, 0.0, 1.0]),
            }
            self.get_logger().info(f'Added satellite nasa_satellite3 with CSV file: {csv_file3}')
        
        if not self.satellites:
            self.get_logger().error('No satellite CSV mappings provided!')
            return
        
        # 서비스 클라이언트 생성: SetEntityPose 서비스 호출
        self.set_pose_client = self.create_client(SetEntityPose, '/world/space_world/set_pose')
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_pose service...')
        self.get_logger().info('set_pose service is ready.')
        
        # 디버깅을 위한 시간 추적
        self.start_time = self.get_clock().now()
        self.last_log_time = 0.0
        self.first_update_time = None
        self.last_update_time = None
        
        # CSV 데이터를 별도 스레드로 로드
        self.load_thread = threading.Thread(target=self.load_all_csv_data)
        self.load_thread.daemon = True
        self.load_thread.start()
        self.data_loaded = False
        
        # 주기적 업데이트 타이머 생성
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_satellite_positions)
        
        self.get_logger().info(f'Multi-satellite controller guidence initialized for {len(self.satellites)} satellites with update rate {self.update_rate} Hz.')
    
    def load_all_csv_data(self):
        """모든 위성의 CSV 데이터를 로드"""
        for sat_name, sat_info in self.satellites.items():
            self.get_logger().info(f'Loading CSV data for {sat_name} from {sat_info["csv_path"]}')
            try:
                with open(sat_info['csv_path'], 'r') as csvfile:
                    reader = csv.DictReader(csvfile)
                    for row in reader:
                        sat_info['data']['time_index'].append(float(row['Time_Index']))
                        sat_info['data']['x'].append(float(row['x']))
                        sat_info['data']['vx'].append(float(row['vx']))
                        sat_info['data']['y'].append(float(row['y']))
                        sat_info['data']['vy'].append(float(row['vy']))
                        sat_info['data']['z'].append(float(row['z']))
                        sat_info['data']['vz'].append(float(row['vz']))
                self.create_interpolators(sat_name, sat_info)
            except Exception as e:
                self.get_logger().error(f'Failed to load CSV for {sat_name}: {e}')
        self.data_loaded = True
        self.get_logger().info('All CSV data loaded successfully.')
    
    def create_interpolators(self, sat_name, sat_info):
        """위성 데이터에 대한 보간 함수 생성"""
        data = sat_info['data']
        sat_info['time_min'] = min(data['time_index'])
        sat_info['time_max'] = max(data['time_index'])
        sat_info['duration'] = sat_info['time_max'] - sat_info['time_min']
        
        sat_info['interpolators']['x'] = interp1d(data['time_index'], data['x'], bounds_error=False, fill_value=(data['x'][0], data['x'][-1]))
        sat_info['interpolators']['vx'] = interp1d(data['time_index'], data['vx'], bounds_error=False, fill_value=(data['vx'][0], data['vx'][-1]))
        sat_info['interpolators']['y'] = interp1d(data['time_index'], data['y'], bounds_error=False, fill_value=(data['y'][0], data['y'][-1]))
        sat_info['interpolators']['vy'] = interp1d(data['time_index'], data['vy'], bounds_error=False, fill_value=(data['vy'][0], data['vy'][-1]))
        sat_info['interpolators']['z'] = interp1d(data['time_index'], data['z'], bounds_error=False, fill_value=(data['z'][0], data['z'][-1]))
        sat_info['interpolators']['vz'] = interp1d(data['time_index'], data['vz'], bounds_error=False, fill_value=(data['vz'][0], data['vz'][-1]))
    
    def calculate_quaternion_for_camera_pointing(self, position):
        """위치에 기반한 카메라 방향 쿼터니언 계산"""
        target = np.array([0.0, 0.0, 0.0])  # 지구 중심을 향함
        direction = target - np.array(position)
        norm = np.linalg.norm(direction)
        if norm == 0:
            return 0.0, 0.0, 0.0, 1.0
        direction /= norm
        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(up, direction)
        right_norm = np.linalg.norm(right)
        if right_norm == 0:
            right = np.array([1.0, 0.0, 0.0])
        else:
            right /= right_norm
        up = np.cross(direction, right)
        rot_matrix = np.vstack([right, up, direction]).T
        r = Rotation.from_matrix(rot_matrix)
        q = r.as_quat()
        return q[0], q[1], q[2], q[3]
    
    def update_satellite_positions(self):
        """위성 위치 업데이트 (Gazebo 업데이트)"""
        if not self.data_loaded:
            return
        
        current_time_obj = self.get_clock().now()
        
        # 첫 업데이트 시간 기록
        if self.first_update_time is None:
            self.first_update_time = current_time_obj
            self.last_update_time = current_time_obj
        
        # 실시간 동기화를 위한 시간 계산
        elapsed_real_time = (current_time_obj - self.first_update_time).nanoseconds / 1e9
        
        # 타이머 콜백 간의 실제 간격 계산 (ms 단위)
        dt_ms = (current_time_obj - self.last_update_time).nanoseconds / 1e6
        self.last_update_time = current_time_obj
        
        # 주기적인 로깅
        current_real_time = (current_time_obj - self.start_time).nanoseconds / 1e9
        if current_real_time - self.last_log_time > 30.0:
            self.get_logger().info(
                f'시뮬레이션 진행: {elapsed_real_time:.1f}s, 타이머 간격: {dt_ms:.1f}ms'
            )
            self.last_log_time = current_real_time
        
        # 스레드 안전성을 위한 락 사용
        with self.state_lock:
            for sat_name, sat_info in self.satellites.items():
                interpolators = sat_info.get('interpolators', {})
                if not interpolators:
                    self.get_logger().warn(f'No interpolators for {sat_name}, skipping update.')
                    continue
                
                # 시뮬레이션 시간 계산
                time_min = sat_info.get('time_min', 0)
                time_max = sat_info.get('time_max', 0)
                duration = sat_info.get('duration', 1.0)
                
                # 실시간으로 CSV 데이터 인덱싱을 위한 시간 계산
                data_time = elapsed_real_time * self.time_scale
                
                if self.loop_data:
                    current_time = time_min + (data_time % duration)
                else:
                    current_time = min(time_min + data_time, time_max)
                
                # 보간된 위치 계산
                try:
                    x = float(interpolators['x'](current_time))
                    y = float(interpolators['y'](current_time))
                    z = float(interpolators['z'](current_time))
                    
                    # 속도 직접 계산 (보간된 값 사용, 유도 목적에서는 사용하지 않음)
                    vx = float(interpolators['vx'](current_time))
                    vy = float(interpolators['vy'](current_time))
                    vz = float(interpolators['vz'](current_time))
                    
                    # 현재 상태 업데이트
                    sat_info['current_position'] = np.array([x, y, z])
                    sat_info['current_velocity'] = np.array([vx, vy, vz])
                    
                    # 위성 자세 계산
                    qx, qy, qz, qw = self.calculate_quaternion_for_camera_pointing([x, y, z])
                    sat_info['current_orientation'] = np.array([qx, qy, qz, qw])
                    
                    # 현재 시간 저장
                    sat_info['current_time'] = current_time
                    
                    # 서비스 요청 메시지 구성
                    req = SetEntityPose.Request()
                    req.entity.name = sat_name
                    req.pose.position.x = x
                    req.pose.position.y = y
                    req.pose.position.z = z
                    req.pose.orientation.x = qx
                    req.pose.orientation.y = qy
                    req.pose.orientation.z = qz
                    req.pose.orientation.w = qw
                    
                    # 서비스 비동기 호출
                    future = self.set_pose_client.call_async(req)
                    future.add_done_callback(lambda fut, name=sat_name: self.handle_set_pose_result(fut, name))
                    
                except Exception as e:
                    self.get_logger().error(f'Error updating {sat_name} position: {e}')
    
    def handle_set_pose_result(self, future, sat_name):
        """서비스 호출 결과 처리"""
        try:
            response = future.result()
            self.get_logger().info(f'Successfully updated pose for {sat_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to update pose for {sat_name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    controller = MultiSatelliteControllerGuidence()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()