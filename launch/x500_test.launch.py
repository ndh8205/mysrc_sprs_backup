from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ


def generate_launch_description():
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'x500_test.world')
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
            '/world/x500_test_world/model/x500/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    return LaunchDescription([
        start_world,
        imu_bridge,
    ])
