#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import math
import threading
import numpy as np
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose, TransformStamped
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class MultiSatelliteControllerService(Node):
    def __init__(self):
        super().__init__('multi_satellite_controller_service')
        
        # 파라미터 선언
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('loop_data', True)
        self.declare_parameter('csv_file1', '')
        self.declare_parameter('csv_file2', '')
        self.declare_parameter('csv_file3', '')
        self.declare_parameter('csv_data_rate', 50.0)  # CSV 데이터의 실제 샘플링 레이트 (Hz)
        self.declare_parameter('time_scale', 1.0)  # 시간 스케일 팩터 (1.0 = 실시간, 2.0 = 2배 빠르게)
        self.declare_parameter('tf_entity', '')  # TF 발행할 위성 이름 (예: 'nasa_satellite3')
        self.declare_parameter('tf_lidar_frame', '')  # LiDAR 프레임 (예: 'nasa_satellite3/nasa_satellite_link/lidar_3d')
        
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
        
        self.tf_entity = self.get_parameter('tf_entity').get_parameter_value().string_value
        self.tf_lidar_frame = self.get_parameter('tf_lidar_frame').get_parameter_value().string_value

        self.get_logger().info(f'CSV 데이터 간격: {self.csv_time_step}초 ({self.csv_data_rate}Hz)')
        self.get_logger().info(f'시간 스케일 팩터: {self.time_scale}x')

        # TF broadcaster (tf_entity가 지정된 경우에만 활성화)
        self.tf_broadcaster = None
        if self.tf_entity:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info(f'TF 발행 활성화: odom -> base_link ({self.tf_entity})')
            if self.tf_lidar_frame:
                static_broadcaster = StaticTransformBroadcaster(self)
                static_tf = TransformStamped()
                static_tf.header.stamp = self.get_clock().now().to_msg()
                static_tf.header.frame_id = 'base_link'
                static_tf.child_frame_id = self.tf_lidar_frame
                # nasa_satellite3 SDF LiDAR pose: (-0.11677, 0.5224, 0.2269, 0, 0, π/2)
                static_tf.transform.translation.x = -0.11677
                static_tf.transform.translation.y = 0.5224
                static_tf.transform.translation.z = 0.2269
                static_tf.transform.rotation.z = 0.7071068
                static_tf.transform.rotation.w = 0.7071068
                static_broadcaster.sendTransform(static_tf)
                self.get_logger().info(f'Static TF: base_link -> {self.tf_lidar_frame}')
        
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
                'interpolators': {}
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
                'interpolators': {}
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
                'interpolators': {}
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
        
        # 정확한 실시간 동기화를 위한 마지막 업데이트 시간
        self.last_update_time = None
        
        # CSV 데이터를 별도 스레드로 로드
        self.load_thread = threading.Thread(target=self.load_all_csv_data)
        self.load_thread.daemon = True
        self.load_thread.start()
        self.data_loaded = False
        
        # 주기적 업데이트 타이머 생성
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_satellite_positions)
        self.get_logger().info(f'Multi-satellite controller service initialized for {len(self.satellites)} satellites with update rate {self.update_rate} Hz.')
    
    def load_all_csv_data(self):
        for sat_name, sat_info in self.satellites.items():
            self.get_logger().info(f'Loading CSV data for {sat_name} from {sat_info["csv_path"]}')
            self.load_csv_data(sat_name, sat_info)
        self.data_loaded = True
        self.get_logger().info('All CSV data loaded successfully.')
    
    def load_csv_data(self, sat_name, sat_info):
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
                    self.get_logger().error(f'CSV format error for {sat_name}. Fields: {fieldnames}')
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
                
                # 첫 10개 타임 인덱스 값 출력 (디버깅용)
                if row_count > 10:
                    first_10_indices = sat_info['data']['time_index'][:10]
                    self.get_logger().info(f'First 10 time indices for {sat_name}: {first_10_indices}')
                
                self.get_logger().info(f'Loaded {row_count} rows for {sat_name}')
                
                # 보간 함수 생성
                self.create_interpolators(sat_name, sat_info)
                
        except Exception as e:
            self.get_logger().error(f'Error loading CSV data for {sat_name}: {e}')
    
    def create_interpolators(self, sat_name, sat_info):
        """CSV 데이터에 대한 보간 함수 생성"""
        data = sat_info['data']
        if len(data['time_index']) < 2:
            self.get_logger().error(f'Not enough data for interpolation for {sat_name}')
            return
        
        # 시간 배열 생성 (타임스탬프를 초 단위로 변환)
        # CSV 파일의 타임 인덱스에 실제 시간 간격 적용
        time_array = np.array(data['time_index']) * self.csv_time_step
        
        # 최대/최소 시간 확인
        min_time = time_array[0]
        max_time = time_array[-1]
        self.get_logger().info(f'{sat_name} 시간 범위: {min_time:.2f}s ~ {max_time:.2f}s (총 {max_time-min_time:.2f}s)')
        
        # 각 변수에 대한 보간 함수 생성 (선형 보간)
        interpolators = {}
        for key in ['x', 'y', 'z', 'vx', 'vy', 'vz']:
            if len(data[key]) >= 2:  # 보간에는 최소 2개의 점이 필요
                interpolators[key] = interp1d(
                    time_array, 
                    np.array(data[key]), 
                    kind='linear',  # 선형 보간
                    bounds_error=False,  # 범위를 벗어나도 오류 발생하지 않음
                    fill_value=(data[key][0], data[key][-1])  # 범위 밖은 첫/마지막 값으로 설정
                )
            else:
                self.get_logger().warn(f'Not enough data for {key} interpolation for {sat_name}')
                
        sat_info['interpolators'] = interpolators
        sat_info['time_min'] = min_time
        sat_info['time_max'] = max_time
        sat_info['duration'] = max_time - min_time
        self.get_logger().info(f'Created interpolators for {sat_name}, duration: {sat_info["duration"]:.2f}s')
    
    def calculate_quaternion_for_camera_pointing(self, position):
        """
        위성의 위치를 기반으로 카메라가 원점(0,0,0)을 직접 바라보도록 하는 쿼터니언을 계산합니다.
        """
        # 위성에서 원점으로의 방향 벡터
        to_origin = np.array([-position[0], -position[1], -position[2]])
        
        # 방향 벡터 정규화
        direction_norm = np.linalg.norm(to_origin)
        if direction_norm < 1e-6:  # 원점에 너무 가까운 경우 처리
            return 0.0, 0.0, 0.0, 1.0  # 기본 쿼터니언 (회전 없음)
        
        to_origin = to_origin / direction_norm
        
        # 카메라가 Y축 방향을 따라 바라보기 때문에, 
        # 위성의 Y축이 원점 방향을 가리키도록 회전 계산
        
        # 회전 행렬 계산을 위한 축 설정
        y_axis = to_origin  # Y축은 원점 방향
        
        # Z축 계산 (위쪽 방향, 가능하면 세계 Z축과 정렬)
        world_up = np.array([0.0, 0.0, 1.0])
        
        # Y축과 세계 Z축이 거의 평행한 경우 처리
        if abs(np.dot(y_axis, world_up)) > 0.99:
            # 대체 up 벡터 사용
            world_up = np.array([0.0, 1.0, 0.0])
            if abs(np.dot(y_axis, world_up)) > 0.99:
                world_up = np.array([1.0, 0.0, 0.0])
        
        # X축 계산 (Y축과 world_up의 외적)
        x_axis = np.cross(y_axis, world_up)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Z축 계산 (X축과 Y축의 외적으로 직교 기저 완성)
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)
        
        # 회전 행렬 구성 (위성 본체 기준)
        rotation_matrix = np.zeros((3, 3))
        rotation_matrix[:, 0] = x_axis
        rotation_matrix[:, 1] = y_axis  # Y축이 원점 방향
        rotation_matrix[:, 2] = z_axis
        
        # 회전 행렬을 쿼터니언으로 변환
        rot = Rotation.from_matrix(rotation_matrix)
        
        # 최종 쿼터니언 추출
        quat = rot.as_quat()  # x, y, z, w 순서
        
        return quat[0], quat[1], quat[2], quat[3]
    
    def update_satellite_positions(self):
        if not self.data_loaded:
            return
        
        current_time_obj = self.get_clock().now()
        
        # 첫 업데이트 시간 기록 (첫 호출 시에만)
        if self.first_update_time is None:
            self.first_update_time = current_time_obj
            self.last_update_time = current_time_obj
            self.get_logger().info(f'First update at: {current_time_obj.nanoseconds/1e9:.6f}s')
        
        # 정확한 실시간 동기화를 위한 시간 계산
        elapsed_real_time = (current_time_obj - self.first_update_time).nanoseconds / 1e9
        
        # 타이머 콜백 간의 실제 간격 계산 (ms 단위)
        dt_ms = (current_time_obj - self.last_update_time).nanoseconds / 1e6
        self.last_update_time = current_time_obj
        
        # 주기적인 로깅 (5초마다)
        current_real_time = (current_time_obj - self.start_time).nanoseconds / 1e9
        if current_real_time - self.last_log_time > 5.0:
            self.get_logger().info(
                f'실제 시간: {elapsed_real_time:.3f}s, 타이머 간격: {dt_ms:.2f}ms, '
                f'CSV 시간 스케일: {self.time_scale}x'
            )
            self.last_log_time = current_real_time
        
        for sat_name, sat_info in self.satellites.items():
            interpolators = sat_info.get('interpolators', {})
            if not interpolators:
                self.get_logger().warn(f'No interpolators for {sat_name}, skipping update.')
                continue
            
            # 시뮬레이션 시간 (실제 시간)에 time_scale을 적용한 CSV 데이터 시간 계산
            time_min = sat_info.get('time_min', 0)
            time_max = sat_info.get('time_max', 0)
            duration = sat_info.get('duration', 1.0)
            
            # 실시간으로 CSV 데이터 인덱싱을 위한 시간 계산
            data_time = elapsed_real_time * self.time_scale
            
            if self.loop_data:
                # 루프 활성화: 데이터의 끝에 도달하면 처음부터 다시 시작
                current_time = time_min + (data_time % duration)
            else:
                # 루프 비활성화: 데이터의 끝에 도달하면 마지막 값 유지
                current_time = min(time_min + data_time, time_max)
            
            # 보간된 위치 계산
            try:
                x = float(interpolators['x'](current_time))
                y = float(interpolators['y'](current_time))
                z = float(interpolators['z'](current_time))
                
                # 디버그 로그 (10초마다)
                if int(elapsed_real_time) % 10 == 0 and abs(elapsed_real_time % 10) < 0.1:
                    self.get_logger().info(
                        f'{sat_name} position at t={elapsed_real_time:.2f}s '
                        f'(data_t={current_time:.2f}s): '
                        f'({x:.6f}, {y:.6f}, {z:.6f})'
                    )
                
                # 위성 자세 계산 및 위치 설정
                qx, qy, qz, qw = self.calculate_quaternion_for_camera_pointing([x, y, z])
                
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

                # TF 발행 (지정된 위성에 대해서만)
                if self.tf_broadcaster and sat_name == self.tf_entity:
                    t = TransformStamped()
                    t.header.stamp = current_time_obj.to_msg()
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = x
                    t.transform.translation.y = y
                    t.transform.translation.z = z
                    t.transform.rotation.x = qx
                    t.transform.rotation.y = qy
                    t.transform.rotation.z = qz
                    t.transform.rotation.w = qw
                    self.tf_broadcaster.sendTransform(t)
                
            except Exception as e:
                self.get_logger().error(f'Error updating {sat_name} position: {e}')
    
    def handle_set_pose_result(self, future, sat_name):
        try:
            result = future.result()
            if result is not None and result.success:
                pass  # 성공했을 때는 로그를 남기지 않아 로그 오버헤드 감소
            else:
                self.get_logger().warn(f"{sat_name} pose set failed: {result.status_message if result else 'Unknown error'}")
        except Exception as e:
            self.get_logger().error(f"Exception while handling set_pose result for {sat_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    controller = MultiSatelliteControllerService()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()