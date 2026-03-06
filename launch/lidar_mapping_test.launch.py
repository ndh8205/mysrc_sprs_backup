from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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

    # 1. Gazebo
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # 2. LiDAR PointCloud2 브리지
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/controla_prototype_1/lidar/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )

    # 3. SetEntityPose 서비스 브리지
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        arguments=[
            '/world/lidar_test_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen'
    )

    # 4. 궤도 스캐너 (위성 회전 + odom->base_link TF)
    orbit_scanner = Node(
        package='orbit_sim',
        executable='lidar_orbit_scanner',
        name='lidar_orbit_scanner',
        parameters=[{
            'orbit_radius': 4.0,
            'orbit_speed': 0.1,
            'update_rate': 10.0,
        }],
        output='screen'
    )

    # 5. 3D 포인트클라우드 매퍼 (누적 맵 생성)
    pointcloud_mapper = Node(
        package='orbit_sim',
        executable='pointcloud_mapper',
        name='pointcloud_mapper',
        parameters=[{
            'voxel_size': 0.03,
            'max_points': 500000,
            'publish_rate': 2.0,
        }],
        output='screen'
    )

    # 6. RViz2 (3D 시각화)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(orbit_sim_path, 'config', 'lidar_mapping.rviz')],
        output='screen'
    )

    # orbit_scanner와 mapper는 Gazebo 시작 후 5초 지연
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[orbit_scanner, pointcloud_mapper]
    )

    return LaunchDescription([
        start_world,
        lidar_bridge,
        pose_bridge,
        delayed_nodes,
        rviz2,
    ])
