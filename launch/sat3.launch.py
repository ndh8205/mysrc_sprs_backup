from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ

def generate_launch_description():
    # orbit_sim 패키지의 월드 및 모델 파일 경로
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'gco_controla.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    # 환경 변수 설정
    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }

    # 1. Gazebo 시뮬레이션 시작
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # 2. 카메라 브릿지 노드
    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/controla_prototype_1/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/controla_prototype_2/camera@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen'
    )

    # 3. IMU 브릿지 노드 (새로 추가)
    imu_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/controla_prototype_1/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/controla_prototype_2/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    

    entity_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='entity_bridge',
        arguments=[
            '/world/space_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        output='screen'
    )

    # rosbridge 서버 노드 추가
    rosbridge_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'
        }],
        output='screen'
    )

    lidar_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            # 포인트 클라우드 데이터 브릿지
            '/nasa_satellite3/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    return LaunchDescription([
        start_world,
        camera_bridge_node,
        imu_bridge_node,
        #imu_visualizer_node,
        entity_bridge_node,
        rosbridge_server_node,
        lidar_bridge_node,
    ])