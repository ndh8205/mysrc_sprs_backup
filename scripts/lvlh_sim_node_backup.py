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
        self.declare_parameter('attitude_mode', 'efficient')  # 'full' or 'efficient'
        
        # 파라미터 읽기
        self.update_rate = self.get_parameter('update_rate').value
        self.loop_data = self.get_parameter('loop_data').value
        csv_filename = self.get_parameter('csv_file').value
        self.time_scale = self.get_parameter('playback_speed').value  # playback_speed를 time_scale로 사용
        self.chief_name = self.get_parameter('chief_model_name').value
        self.deputy_name = self.get_parameter('deputy_model_name').value
        self.attitude_mode = self.get_parameter('attitude_mode').value
        
        # CSV 파일 경로
        package_dir = get_package_share_directory('orbit_sim')
        csv_path = os.path.join(package_dir, 'data', csv_filename)
        
        self.get_logger().info(f'시간 스케일 팩터: {self.time_scale}x')
        self.get_logger().info(f'자세 제어 모드: {self.attitude_mode}')
        
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
                    'x': [], 'y': [], 'z': [],
                    'vx': [], 'vy': [], 'vz': []  # 속도 데이터 추가
                },
                'interpolators': {},
                'is_chief': False
            }
        }
        
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
                lvlh_vx_data = []
                lvlh_vy_data = []
                lvlh_vz_data = []
                
                row_count = 0
                for row in reader:
                    time_data.append(float(row['time']))
                    lvlh_x_data.append(float(row['lvlh_x']))  # km 단위
                    lvlh_y_data.append(float(row['lvlh_y']))
                    lvlh_z_data.append(float(row['lvlh_z']))
                    
                    # 속도 데이터 추가 (km/s 단위)
                    lvlh_vx_data.append(float(row['lvlh_vx']))
                    lvlh_vy_data.append(float(row['lvlh_vy']))
                    lvlh_vz_data.append(float(row['lvlh_vz']))
                    
                    row_count += 1
                
                self.get_logger().info(f'Loaded {row_count} rows')
                
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
                deputy_info['data']['vx'] = [vx * 1000.0 for vx in lvlh_vx_data]  # km/s -> m/s
                deputy_info['data']['vy'] = [vy * 1000.0 for vy in lvlh_vy_data]
                deputy_info['data']['vz'] = [vz * 1000.0 for vz in lvlh_vz_data]
                
                # 보간 함수 생성
                self.create_interpolators(self.chief_name, chief_info)
                self.create_interpolators(self.deputy_name, deputy_info)
                
        except Exception as e:
            self.get_logger().error(f'Error loading CSV data: {e}')
            
        self.data_loaded = True
        self.get_logger().info('All CSV data loaded successfully.')
    
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
        
        # 선형 보간 함수 생성 (속도 포함)
        interpolators = {}
        for key in ['x', 'y', 'z', 'vx', 'vy', 'vz']:
            # Chief는 속도 데이터가 없을 수 있음
            if key in data and len(data[key]) > 0:
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
        Full 3축 제어 모드: 위성의 위치를 기반으로 카메라가 원점(0,0,0)을 직접 바라보도록 하는 쿼터니언을 계산합니다.
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
    
    def calculate_quaternion_efficient_mode(self, position, velocity):
        """
        GCO 궤도면에 Z축을 정렬하고, Y축이 원점(Chief)을 향하도록 자세를 계산합니다.
        
        Args:
            position (list or np.array): Deputy의 LVLH 상대 위치 [x, y, z] (단위: m)
            velocity (list or np.array): Deputy의 LVLH 상대 속도 [vx, vy, vz] (단위: m/s)
        """
        rel_pos = np.array(position)
        rel_vel = np.array(velocity)

        # 수학적 안정성을 위한 예외 처리
        pos_norm = np.linalg.norm(rel_pos)
        if pos_norm < 1e-6:  # 위치가 거의 원점일 경우
            return 0.0, 0.0, 0.0, 1.0  # 기본 자세 (회전 없음)

        # 1. GCO 궤도면의 법선 벡터 (공전 축) 계산
        # 각운동량 벡터 h = r x v
        angular_momentum_vec = np.cross(rel_pos, rel_vel)
        h_norm = np.linalg.norm(angular_momentum_vec)
        
        if h_norm < 1e-6:  # 궤도면이 정의되지 않는 경우 (직선 운동 등)
            # 대안으로 'full' 모드와 유사하게 동작
            return self.calculate_quaternion_for_camera_pointing(position)

        # GCO 궤도의 법선 벡터 = 위성의 목표 Z축
        sat_z_axis = angular_momentum_vec / h_norm

        # 2. 목표 Y축 계산
        # Y축은 원점을 향해야 함. 목표 방향 벡터 정의.
        y_axis_target_dir = -rel_pos / pos_norm
        
        # 3. 직교하는 자세 행렬(Rotation Matrix) 구성
        # 목표 Y축은 반드시 목표 Z축(sat_z_axis)과 수직이어야 함.
        # Gram-Schmidt 과정과 유사하게 y_axis_target_dir을 sat_z_axis에 대해 직교화.
        # 먼저, 두 벡터를 이용해 X축을 정의 (X축은 Y, Z축 모두와 수직)
        sat_x_axis = np.cross(y_axis_target_dir, sat_z_axis)
        
        # X축이 0벡터가 되는 경우 (y_axis_target_dir과 sat_z_axis가 평행할 때)
        # 매우 드문 경우이지만, 발생 시 자세를 정의할 수 없으므로 대체 벡터 사용
        if np.linalg.norm(sat_x_axis) < 1e-6:
            # 대체 벡터로 LVLH의 X축 사용
            alternative_vec = np.array([1.0, 0.0, 0.0])
            sat_x_axis = np.cross(alternative_vec, sat_z_axis)
            # 그래도 0벡터이면 LVLH의 Y축 사용
            if np.linalg.norm(sat_x_axis) < 1e-6:
                alternative_vec = np.array([0.0, 1.0, 0.0])
                sat_x_axis = np.cross(alternative_vec, sat_z_axis)

        sat_x_axis = sat_x_axis / np.linalg.norm(sat_x_axis)

        # 이제 Z축과 X축을 사용하여 완벽하게 직교하는 Y축을 재계산
        sat_y_axis = np.cross(sat_z_axis, sat_x_axis)

        # 가제보 축계 및 모델링 가정
        # 아래 회전 행렬은 위성 모델의 X, Y, Z축이
        # 각각 sat_x_axis, sat_y_axis, sat_z_axis와 정렬되도록 합니다.
        # 만약 위성 모델(SDF)의 카메라가 +Y축이 아닌 +X축을 향한다면,
        # rotation_matrix의 첫 번째 열과 두 번째 열을 바꿔주어야 합니다.
        # (rotation_matrix[:, 0] = sat_y_axis, rotation_matrix[:, 1] = -sat_x_axis)
        
        rotation_matrix = np.zeros((3, 3))
        rotation_matrix[:, 0] = sat_x_axis
        rotation_matrix[:, 1] = sat_y_axis  # Y축이 원점을 최대한 향함
        rotation_matrix[:, 2] = sat_z_axis  # Z축이 GCO 공전 축에 정렬됨
        
        # 4. 회전 행렬을 쿼터니언으로 변환
        rot = Rotation.from_matrix(rotation_matrix)
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
            
            # 보간된 위치 및 속도
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
                
                # 위성 자세 계산 (모드에 따라 다른 방식 사용)
                if self.attitude_mode == 'efficient':
                    # 속도 데이터도 보간하여 함수에 전달
                    vx = float(interpolators['vx'](current_time))
                    vy = float(interpolators['vy'](current_time))
                    vz = float(interpolators['vz'](current_time))
                    qx, qy, qz, qw = self.calculate_quaternion_efficient_mode([x, y, z], [vx, vy, vz])
                else:  # 'full' mode
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