from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ

def generate_launch_description():
    # orbit_sim 패키지의 월드 및 모델 파일 경로
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'gco_test.world')
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
            '/nasa_satellite/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/nasa_satellite2/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/nasa_satellite3/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/stereo/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/stereo/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )

    # 3. IMU 브릿지 노드 (새로 추가)
    imu_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/nasa_satellite/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/nasa_satellite2/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/nasa_satellite3/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/nasa_satellite4/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        output='screen'
    )

    # 4. IMU Visualizer 노드 (새로 추가)
    #imu_visualizer_node = Node(
    #    package='orbit_sim',
    #    executable='imu_visualizer',
    #    name='imu_visualizer',
    #    parameters=[{
    #        'satellite_name': 'nasa_satellite2',
    #        'history_length': 100,
    #        'update_rate': 10.0,
    #        'quaternion_order': 'xyzw',  # ROS 표준은 x,y,z,w
    #        'quaternion_convention': 'hamilton'  # ROS 표준은 Hamilton convention
    #    }],
    #    output='screen'
    #)

    return LaunchDescription([
        start_world,
        camera_bridge_node,
        imu_bridge_node,
    ])