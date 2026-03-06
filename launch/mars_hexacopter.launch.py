from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ


def generate_launch_description():
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'mars_hexacopter.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }

    # Gazebo simulation
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # ROS-Gazebo bridge: IMU
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/world/mars_hexacopter_world/model/mars_hexacopter/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge: CMG joint states (GZ -> ROS)
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/mars_hexacopter/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge: CMG gimbal velocity commands (ROS -> GZ)
    gimbal_cmd_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gimbal_cmd_bridge',
        arguments=[
            '/mars_hexacopter/cmg1_gimbal/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/mars_hexacopter/cmg2_gimbal/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge: CMG wheel velocity commands (ROS -> GZ)
    wheel_cmd_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wheel_cmd_bridge',
        arguments=[
            '/mars_hexacopter/cmg1_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/mars_hexacopter/cmg2_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    return LaunchDescription([
        start_world,
        imu_bridge,
        joint_state_bridge,
        gimbal_cmd_bridge,
        wheel_cmd_bridge,
    ])
