from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ


def generate_launch_description():
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'lidar_test.world')
    model_path = os.path.join(orbit_sim_path, 'models')

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

    # 2. LiDAR 브리지
    #    gpu_lidar는 2개 토픽 생성:
    #      {topic}        → gz.msgs.LaserScan (2D 스캔)
    #      {topic}/points → gz.msgs.PointCloudPacked (3D 포인트클라우드)
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/controla_prototype_1/lidar/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/controla_prototype_1/lidar/points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # 3. 카메라 브리지 (시각 확인용)
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/controla_prototype_1/camera@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen'
    )

    # 4. SetEntityPose 서비스 (타겟 위치 변경용)
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        arguments=[
            '/world/lidar_test_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen'
    )

    return LaunchDescription([
        start_world,
        lidar_bridge,
        camera_bridge,
        pose_bridge,
    ])
