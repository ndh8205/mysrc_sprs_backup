import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_orbit_sim = get_package_share_directory('orbit_sim')
    
    # Paths
    world_file = os.path.join(pkg_orbit_sim, 'worlds', 'docking_world.sdf')
    urdf_file = os.path.join(pkg_orbit_sim, 'urdf', 'kari_dual_arm.urdf')
    
    # Load URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Gazebo
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states
            '/world/docking_world/model/kari_dual_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Pose info
            '/world/docking_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Servicer IMU
            '/servicer/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Servicer Docking Cameras
            '/servicer/docking_cam_L/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/servicer/docking_cam_R/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Servicer NFOV Cameras
            '/servicer/nfov_L/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/servicer/nfov_R/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Servicer WFOV Cameras
            '/servicer/wfov_L/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/servicer/wfov_R/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Target satellite sensors
            '/intel_sat_dummy/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/intel_sat_dummy/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # Controller Manager spawner - Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Controller Manager spawner - Effort Controller Left
    effort_controller_L_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller_L', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Controller Manager spawner - Effort Controller Right
    effort_controller_R_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller_R', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay controller spawners until Gazebo starts
    delayed_joint_state_broadcaster = TimerAction(
        period=7.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_effort_controller_L = TimerAction(
        period=10.0,
        actions=[effort_controller_L_spawner]
    )

    delayed_effort_controller_R = TimerAction(
        period=12.5,
        actions=[effort_controller_R_spawner]
    )

    # ========== rosbridge 서버 (MATLAB 통신용) ==========
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

    # ========== web_video_server (카메라 스트림용) ==========
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8080}],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        bridge,
        delayed_joint_state_broadcaster,
        delayed_effort_controller_L,
        delayed_effort_controller_R,
        rosbridge_server_node,
        web_video_server_node,
    ])