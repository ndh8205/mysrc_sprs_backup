from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ


def generate_launch_description():
    src_path = os.path.expanduser('~/space_ros_ws/src/orbit_sim/data')
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'gco_test.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    # gz_dvs_plugin 라이브러리 경로 (DVS 플러그인 로드용)
    try:
        dvs_plugin_prefix = get_package_share_directory('gz_dvs_plugin')
        dvs_lib_path = os.path.normpath(
            os.path.join(dvs_plugin_prefix, '..', '..', 'lib'))
    except Exception:
        dvs_lib_path = ''

    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ]),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join(filter(None, [
            environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            dvs_lib_path,
        ])),
    }

    # Launch 파라미터
    csv_file1_arg = DeclareLaunchArgument(
        'csv_file1', default_value=os.path.join(src_path, 'sat1_state.csv'))
    csv_file2_arg = DeclareLaunchArgument(
        'csv_file2', default_value=os.path.join(src_path, 'sat3_state.csv'))
    csv_file3_arg = DeclareLaunchArgument(
        'csv_file3', default_value=os.path.join(src_path, 'sat4_state.csv'))
    time_scale_arg = DeclareLaunchArgument(
        'time_scale', default_value='1.0')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false')

    # 1. Gazebo
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # 2. LiDAR PointCloud2 브리지
    #    nasa_satellite3 gpu_lidar: topic=lidar/points_raw → GZ topic=lidar/points_raw/points
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/lidar/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )

    # 3. SetEntityPose 서비스 브리지
    entity_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='entity_bridge',
        arguments=[
            '/world/space_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen'
    )

    # 4. 멀티 위성 컨트롤러 (CSV 궤적 + nasa_satellite3 TF 발행)
    satellite_controller = Node(
        package='orbit_sim',
        executable='multi_satellite_controller_service',
        name='multi_satellite_controller_service',
        output='screen',
        parameters=[{
            'csv_file1': LaunchConfiguration('csv_file1'),
            'csv_file2': LaunchConfiguration('csv_file2'),
            'csv_file3': LaunchConfiguration('csv_file3'),
            'update_rate': 50.0,
            'csv_data_rate': 50.0,
            'loop_data': True,
            'time_scale': LaunchConfiguration('time_scale'),
            'tf_entity': 'nasa_satellite3',
            'tf_lidar_frame': 'nasa_satellite3/nasa_satellite_link/lidar_3d',
        }]
    )

    # 5. 3D 포인트클라우드 매퍼
    pointcloud_mapper = Node(
        package='orbit_sim',
        executable='pointcloud_mapper',
        name='pointcloud_mapper',
        parameters=[{
            'input_topic': '/lidar/points_raw/points',
            'voxel_size': 0.05,
            'max_points': 500000,
            'publish_rate': 2.0,
        }],
        output='screen'
    )

    # 6. RViz2 (rviz:=true 로 활성화)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(orbit_sim_path, 'config', 'lidar_mapping.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # 컨트롤러와 매퍼는 Gazebo 시작 후 5초 지연
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[satellite_controller, pointcloud_mapper]
    )

    return LaunchDescription([
        csv_file1_arg,
        csv_file2_arg,
        csv_file3_arg,
        time_scale_arg,
        rviz_arg,
        start_world,
        lidar_bridge,
        entity_bridge,
        delayed_nodes,
        rviz2,
    ])
