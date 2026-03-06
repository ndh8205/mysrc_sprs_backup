from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ


def generate_launch_description():
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'mars_hexacopter_cmg500_zerog.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }

    # Gazebo simulation (zero-gravity world)
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
            '/world/mars_hexacopter_cmg500_world/model/mars_hexacopter_cmg500/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge: CMG joint states (GZ -> ROS)
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/mars_hexacopter_cmg500/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge: CMG gimbal velocity commands (ROS -> GZ)
    gimbal_cmd_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gimbal_cmd_bridge',
        arguments=[
            '/mars_hexacopter_cmg500/cmg1_gimbal/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/mars_hexacopter_cmg500/cmg2_gimbal/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge: CMG wheel velocity commands (ROS -> GZ)
    wheel_cmd_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wheel_cmd_bridge',
        arguments=[
            '/mars_hexacopter_cmg500/cmg1_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/mars_hexacopter_cmg500/cmg2_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    # CMG GUI commander (parametrized for CMG500)
    cmg_commander = Node(
        package='orbit_sim',
        executable='mars_hexacopter_cmg_commander',
        name='mars_hexacopter_cmg500_commander',
        parameters=[{
            'model_name': 'mars_hexacopter_cmg500',
            'max_rpm': 30000,
        }],
        output='screen'
    )

    return LaunchDescription([
        start_world,
        imu_bridge,
        joint_state_bridge,
        gimbal_cmd_bridge,
        wheel_cmd_bridge,
        cmg_commander,
    ])
