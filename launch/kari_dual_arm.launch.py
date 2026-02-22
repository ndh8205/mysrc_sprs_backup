import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_orbit_sim = get_package_share_directory('orbit_sim')
    
    # Paths
    world_file = os.path.join(pkg_orbit_sim, 'worlds', 'kari_dual_arm.sdf')
    urdf_file = os.path.join(pkg_orbit_sim, 'urdf', 'kari_dual_arm.urdf')
    
    # Model path for Gazebo
    models_path = os.path.join(pkg_orbit_sim, 'models')
    
    # Load URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Gazebo with model path
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': models_path}
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/kari_dual_arm_world/model/kari_dual_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/kari_dual_arm_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
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

    # Controller Manager spawner - Left Arm Effort Controller
    effort_controller_L_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller_L', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Controller Manager spawner - Right Arm Effort Controller
    effort_controller_R_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller_R', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay controller spawners
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_effort_controller_L = TimerAction(
        period=4.0,
        actions=[effort_controller_L_spawner]
    )

    delayed_effort_controller_R = TimerAction(
        period=4.5,
        actions=[effort_controller_R_spawner]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        bridge,
        delayed_joint_state_broadcaster,
        delayed_effort_controller_L,
        delayed_effort_controller_R,
    ])
