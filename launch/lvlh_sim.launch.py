#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ


def generate_launch_description():
    # Get orbit_sim package paths
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'orbit_GEO.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    # Set environment variables
    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }

    # ========================================
    # Launch Arguments
    # ========================================
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='orbit_data_1hz.csv',
        description='CSV file name for LVLH simulation'
    )
    
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='10.0',
        description='LVLH playback speed multiplier'
    )
    
    attitude_mode_arg = DeclareLaunchArgument(
        'attitude_mode',
        default_value='full',  # Options: 'efficient' (Z-axis only) or 'full' (3-axis control)
        description='Attitude control mode: "efficient" (Z-axis only) or "full" (3-axis control)'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='60.0',
        description='Update rate for satellite position/attitude (Hz)'
    )

    # ========================================
    # Node Definitions
    # ========================================
    
    # 1. Gazebo Simulation
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # 2. Camera Bridge Node
    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/nasa_satellite6/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_nfov/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_nfov/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_wfov/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_wfov/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_docking/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_docking/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen'
    )

    # 3. IMU Bridge Node
    imu_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/nasa_satellite6/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    # 4. Entity Bridge Node
    entity_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='entity_bridge',
        arguments=[
            '/world/space_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        output='screen'
    )

    # 5. ROS Bridge WebSocket Server
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

    # 6. Web Video Server Node
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server_node',
        parameters=[{
            'port': 8080
        }],
        output='screen'
    )

    # 7. LVLH Simulation Node
    lvlh_sim_node = Node(
        package='orbit_sim',
        executable='lvlh_sim_node',
        name='lvlh_sim',
        output='screen',
        parameters=[{
            'csv_file': LaunchConfiguration('csv_file'),
            'playback_speed': LaunchConfiguration('playback_speed'),
            'update_rate': LaunchConfiguration('update_rate'),
            'loop_data': True,
            'chief_model_name': 'intel_sat_dummy',
            'deputy_model_name': 'nasa_satellite6'
        }]
    )

    # ========================================
    # Launch Description
    # ========================================
    return LaunchDescription([
        # Launch arguments
        csv_file_arg,
        playback_speed_arg,
        attitude_mode_arg,
        update_rate_arg,
        
        # Nodes
        start_world,
        camera_bridge_node,
        imu_bridge_node,
        entity_bridge_node,
        rosbridge_server_node,
        web_video_server_node,
        lvlh_sim_node,
    ])