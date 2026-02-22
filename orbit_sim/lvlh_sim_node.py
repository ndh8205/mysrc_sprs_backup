import rclpy
from rclpy.node import Node
import csv
import os
import threading
import numpy as np
from ros_gz_interfaces.srv import SetEntityPose
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

class LVLHSimNode(Node):
    def __init__(self):
        super().__init__('lvlh_sim_node')
        
        # 파라미터 선언
        self.declare_parameter('update_rate', 60.0)
        self.declare_parameter('loop_data', True)
        self.declare_parameter('csv_file', 'orbit_data_1hz.csv')
        self.declare_parameter('playback_speed', 100.0)  # time_scale과 동일한 역할
        self.declare_parameter('chief_model_name', 'intel_sat_dummy')
        self.declare_parameter('deputy_model_name', 'nasa_satellite6')
        
        # 파라미터 읽기
        self.update_rate = self.get_parameter('update_rate').value
        self.loop_data = self.get_parameter('loop_data').value
        csv_filename = self.get_parameter('csv_file').value
        self.time_scale = self.get_parameter('playback_speed').value  # playback_speed를 time_scale로 사용
        self.chief_name = self.get_parameter('chief_model_name').value
        self.deputy_name = self.get_parameter('deputy_model_name').value
        
        # CSV 파일 경로
        package_dir = get_package_share_directory('orbit_sim')
        csv_path = os.path.join(package_dir, 'data', csv_filename)
        
        self.get_logger().info(f'시간 스케일 팩터: {self.time_scale}x')
        
        # 위성 데이터 구조 (multi_satellite_controller와 동일)
        self.satellites = {
            self.chief_name: {
                'csv_path': csv_path,
                'data': {
                    'time': [],
                    'x': [], 'y': [], 'z': []
                },
                'interpolators': {},
                'is_chief': True  # Chief 플래그
            },
            self.deputy_name: {
                'csv_path': csv_path,
                'data': {
                    'time': [],
                    'x': [], 'y': [], 'z': []
                },
                'interpolators': {},
                'is_chief': False
            }
        }
        
        # 궤도 평면 법선 벡터 (초기화)
        self.orbit_normal_vec = None
        
        # 서비스 클라이언트
        self.set_pose_client = self.create_client(SetEntityPose, '/world/space_world/set_pose')
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_pose service...')
        self.get_logger().info('set_pose service is ready.')
        
        # 시간 추적
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
        self.get_logger().info(f'LVLH controller initialized with update rate {self.update_rate} Hz.')
    
    def load_all_csv_data(self):
        """multi_satellite_controller와 동일한 구조로 데이터 로드"""
        csv_path = self.satellites[self.deputy_name]['csv_path']
        
        self.get_logger().info(f'Loading CSV data from {csv_path}')
        
        try:
            # 데이터 읽기
            with open(csv_path, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                
                time_data = []
                lvlh_x_data = []
                lvlh_y_data = []
                lvlh_z_data = []
                
                row_count = 0
                for row in reader:
                    time_data.append(float(row['time']))
                    lvlh_x_data.append(float(row['lvlh_x']))  # km 단위
                    lvlh_y_data.append(float(row['lvlh_y']))
                    lvlh_z_data.append(float(row['lvlh_z']))
                    row_count += 1
                
                self.get_logger().info(f'Loaded {row_count} rows')
                
                # 궤도 평면 법선 벡터 계산 (MATLAB과 동일한 PCA 방식)
                self.calculate_orbit_normal(lvlh_x_data, lvlh_y_data, lvlh_z_data)
                
                # Chief 데이터 (항상 원점)
                chief_info = self.satellites[self.chief_name]
                chief_info['data']['time'] = time_data
                chief_info['data']['x'] = [0.0] * len(time_data)
                chief_info['data']['y'] = [0.0] * len(time_data)
                chief_info['data']['z'] = [0.0] * len(time_data)
                
                # Deputy 데이터 (LVLH 좌표, km를 m로 변환)
                deputy_info = self.satellites[self.deputy_name]
                deputy_info['data']['time'] = time_data
                deputy_info['data']['x'] = [x * 1000.0 for x in lvlh_x_data]  # km -> m
                deputy_info['data']['y'] = [y * 1000.0 for y in lvlh_y_data]
                deputy_info['data']['z'] = [z * 1000.0 for z in lvlh_z_data]
                
                # 보간 함수 생성
                self.create_interpolators(self.chief_name, chief_info)
                self.create_interpolators(self.deputy_name, deputy_info)
                
        except Exception as e:
            self.get_logger().error(f'Error loading CSV data: {e}')
            
        self.data_loaded = True
        self.get_logger().info('All CSV data loaded successfully.')
    
    def calculate_orbit_normal(self, x_data, y_data, z_data):
        """PCA를 사용하여 궤도 평면 법선 벡터 계산 (MATLAB pca() 함수와 동일)"""
        # 궤도 데이터를 numpy 배열로 변환
        orbit_points = np.column_stack((x_data, y_data, z_data))
        
        # 중심을 원점으로 이동
        mean = np.mean(orbit_points, axis=0)
        centered_points = orbit_points - mean
        
        # 공분산 행렬 계산
        cov_matrix = np.cov(centered_points.T)
        
        # 고유값과 고유벡터 계산
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        
        # 고유값을 작은 순서로 정렬
        idx = eigenvalues.argsort()
        eigenvectors = eigenvectors[:, idx]
        
        # 가장 작은 고유값에 대응하는 고유벡터가 궤도 평면 법선
        self.orbit_normal_vec = eigenvectors[:, 0]
        
        # 일관된 방향 유지 (Z 성분이 양수가 되도록)
        if self.orbit_normal_vec[2] < 0:
            self.orbit_normal_vec = -self.orbit_normal_vec
            
        self.get_logger().info(f'Orbit normal vector: [{self.orbit_normal_vec[0]:.4f}, '
                               f'{self.orbit_normal_vec[1]:.4f}, {self.orbit_normal_vec[2]:.4f}]')
    
    def create_interpolators(self, sat_name, sat_info):
        """보간 함수 생성 (multi_satellite_controller와 동일)"""
        data = sat_info['data']
        if len(data['time']) < 2:
            self.get_logger().error(f'Not enough data for interpolation for {sat_name}')
            return
        
        time_array = np.array(data['time'])
        
        min_time = time_array[0]
        max_time = time_array[-1]
        self.get_logger().info(f'{sat_name} 시간 범위: {min_time:.2f}s ~ {max_time:.2f}s')
        
        # 선형 보간 함수 생성
        interpolators = {}
        for key in ['x', 'y', 'z']:
            interpolators[key] = interp1d(
                time_array,
                np.array(data[key]),
                kind='linear',
                bounds_error=False,
                fill_value=(data[key][0], data[key][-1])
            )
        
        sat_info['interpolators'] = interpolators
        sat_info['time_min'] = min_time
        sat_info['time_max'] = max_time
        sat_info['duration'] = max_time - min_time
        self.get_logger().info(f'Created interpolators for {sat_name}, duration: {sat_info["duration"]:.2f}s')
    
    def calculate_quaternion_for_camera_pointing(self, position):
        """
        MATLAB의 CalculateAttitude_NoRoll 함수와 동일한 로직
        롤을 제거하는 새로운 자세 계산 로직
        """
        if self.orbit_normal_vec is None:
            # 궤도 법선이 계산되지 않은 경우 기본값 사용
            return 0.0, 0.0, 0.0, 1.0
        
        # 위치는 m 단위, km로 변환
        pos = np.array(position) / 1000.0
        target = np.array([0.0, 0.0, 0.0])  # Chief 위치 (원점)
        
        # --- 1. 위성 Body의 좌표축 계산 (LVLH 기준) ---
        # Body Y축 (최우선): 위치 -> 타겟 방향
        body_y = target - pos
        body_y_norm = np.linalg.norm(body_y)
        if body_y_norm < 1e-6:
            return 0.0, 0.0, 0.0, 1.0
        body_y = body_y / body_y_norm
        
        # Body X축 (차선): Y축과 궤도 법선에 모두 수직이어야 궤도 평면에 위치함
        body_x = np.cross(body_y, self.orbit_normal_vec)
        body_x_norm = np.linalg.norm(body_x)
        if body_x_norm < 1e-6:
            # Y와 궤도 법선이 평행한 경우 처리
            if abs(body_y[2]) < 0.9:
                alt_normal = np.array([0, 0, 1])
            else:
                alt_normal = np.array([1, 0, 0])
            body_x = np.cross(body_y, alt_normal)
            body_x = body_x / np.linalg.norm(body_x)
        else:
            body_x = body_x / body_x_norm
        
        # Body Z축 (결과): 오른손 법칙으로 결정. 이 축은 더 이상 고정되지 않음
        body_z = np.cross(body_x, body_y)
        body_z = body_z / np.linalg.norm(body_z)
        
        # 위성의 자세 행렬 (LVLH 좌표계에서 표현된 Body 축들)
        R_body_in_lvlh = np.column_stack((body_x, body_y, body_z))
        
        # 회전 행렬을 쿼터니언으로 변환
        rot = Rotation.from_matrix(R_body_in_lvlh)
        quat = rot.as_quat()  # [x, y, z, w] 순서
        
        return quat[0], quat[1], quat[2], quat[3]
    
    def update_satellite_positions(self):
        """multi_satellite_controller와 동일한 업데이트 로직"""
        if not self.data_loaded:
            return
        
        current_time_obj = self.get_clock().now()
        
        # 첫 업데이트 시간 기록
        if self.first_update_time is None:
            self.first_update_time = current_time_obj
            self.last_update_time = current_time_obj
            self.get_logger().info(f'First update at: {current_time_obj.nanoseconds/1e9:.6f}s')
            
            # Chief를 원점에 설정 (자세 포함)
            self.set_chief_initial_position()
        
        # 실시간 동기화
        elapsed_real_time = (current_time_obj - self.first_update_time).nanoseconds / 1e9
        
        # 타이머 간격 계산
        dt_ms = (current_time_obj - self.last_update_time).nanoseconds / 1e6
        self.last_update_time = current_time_obj
        
        # 주기적 로깅 (5초마다)
        current_real_time = (current_time_obj - self.start_time).nanoseconds / 1e9
        if current_real_time - self.last_log_time > 5.0:
            self.get_logger().info(
                f'실제 시간: {elapsed_real_time:.3f}s, 타이머 간격: {dt_ms:.2f}ms, '
                f'시간 스케일: {self.time_scale}x'
            )
            self.last_log_time = current_real_time
        
        # 모든 위성 업데이트 (Chief와 Deputy 모두)
        for sat_name, sat_info in self.satellites.items():
            if sat_info.get('is_chief', False):
                # Chief는 첫 프레임에서만 설정했으므로 스킵
                continue
                
            interpolators = sat_info.get('interpolators', {})
            if not interpolators:
                continue
            
            time_min = sat_info.get('time_min', 0)
            time_max = sat_info.get('time_max', 0)
            duration = sat_info.get('duration', 1.0)
            
            # 시간 계산
            data_time = elapsed_real_time * self.time_scale
            
            if self.loop_data:
                current_time = time_min + (data_time % duration)
            else:
                current_time = min(time_min + data_time, time_max)
            
            # 보간된 위치
            try:
                x = float(interpolators['x'](current_time))
                y = float(interpolators['y'](current_time))
                z = float(interpolators['z'](current_time))
                
                # 디버그 로그 (10초마다)
                if int(elapsed_real_time) % 10 == 0 and abs(elapsed_real_time % 10) < 0.1:
                    self.get_logger().info(
                        f'{sat_name} position at t={elapsed_real_time:.2f}s: '
                        f'({x:.1f}, {y:.1f}, {z:.1f})m'
                    )
                
                # 위성 자세 계산 (원점을 바라보도록)
                qx, qy, qz, qw = self.calculate_quaternion_for_camera_pointing([x, y, z])
                
                # 서비스 요청
                req = SetEntityPose.Request()
                req.entity.name = sat_name
                req.pose.position.x = x
                req.pose.position.y = y
                req.pose.position.z = z
                req.pose.orientation.x = qx
                req.pose.orientation.y = qy
                req.pose.orientation.z = qz
                req.pose.orientation.w = qw
                
                # 비동기 호출
                future = self.set_pose_client.call_async(req)
                future.add_done_callback(lambda fut, name=sat_name: self.handle_set_pose_result(fut, name))
                
            except Exception as e:
                self.get_logger().error(f'Error updating {sat_name} position: {e}')
    
    def set_chief_initial_position(self):
        """Chief를 원점에 설정 (자세 포함)"""
        # Chief는 원점에 있으므로, 기본 자세를 사용하거나
        # 특정 방향을 바라보도록 설정할 수 있음
        # 여기서는 기본 자세 사용 (필요시 수정 가능)
        req = SetEntityPose.Request()
        req.entity.name = self.chief_name
        req.pose.position.x = 0.0
        req.pose.position.y = 0.0
        req.pose.position.z = 0.0
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = 0.0
        req.pose.orientation.w = 1.0
        
        self.set_pose_client.call_async(req)
        self.get_logger().info(f'{self.chief_name} set to origin (0, 0, 0) with default orientation')
    
    def handle_set_pose_result(self, future, sat_name):
        """결과 처리 (multi_satellite_controller와 동일)"""
        try:
            result = future.result()
            if result is not None and not result.success:
                self.get_logger().warn(f"{sat_name} pose set failed")
        except Exception as e:
            self.get_logger().error(f"Exception for {sat_name}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LVLHSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()