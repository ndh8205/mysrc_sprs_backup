from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ


def generate_launch_description():
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'stereo_test.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }

    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/stereo/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/stereo/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen'
    )

    set_pose_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='set_pose_bridge',
        arguments=[
            '/world/stereo_test_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        output='screen'
    )

    stereo_gui_node = Node(
        package='orbit_sim',
        executable='stereo_test_gui',
        name='stereo_test_gui',
        output='screen'
    )

    return LaunchDescription([
        start_world,
        camera_bridge_node,
        set_pose_bridge_node,
        stereo_gui_node,
    ])
