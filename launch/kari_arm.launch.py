import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_orbit_sim = get_package_share_directory('orbit_sim')
    
    # Paths
    world_file = os.path.join(pkg_orbit_sim, 'worlds', 'kari_arm.sdf')
    urdf_file = os.path.join(pkg_orbit_sim, 'urdf', 'kari_arm.urdf')
    controller_config = os.path.join(pkg_orbit_sim, 'config', 'kari_arm_controllers.yaml')
    
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/kari_arm_world/model/kari_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/kari_arm_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
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

    # Controller Manager spawner - Effort Controller
    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay controller spawners until Gazebo starts
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_effort_controller = TimerAction(
        period=4.0,
        actions=[effort_controller_spawner]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        bridge,
        delayed_joint_state_broadcaster,
        delayed_effort_controller,
    ])
