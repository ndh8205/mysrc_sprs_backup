#!/usr/bin/env python3
"""
Sensor Visualization GCO Launch
- orbit_LVLH_GCO 공전 시나리오 + Gazebo GUI 센서 시각화
- 카메라 / DVS 이벤트 카메라 / LiDAR 포인트 클라우드를 Gazebo에서 직접 확인
- RViz 불필요
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ


def generate_launch_description():
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'sensor_viz_gco.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }

    # Gazebo 시뮬레이션
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # Odometry 브릿지 (GCO 컨트롤러용)
    odometry_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odometry_bridge',
        arguments=[
            '/model/nasa_satellite5/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )

    # IMU 브릿지
    imu_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/nasa_satellite5/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    # SetEntityPose 브릿지
    set_pose_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='set_pose_bridge',
        arguments=[
            '/world/space_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        output='screen'
    )

    # Wrench 브릿지 (GCO 추력 제어용)
    wrench_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wrench_bridge',
        arguments=[
            '/world/space_world/wrench@ros_gz_interfaces/msg/EntityWrench@gz.msgs.EntityWrench'
        ],
        output='screen'
    )

    # GCO 컨트롤러
    gco_controller_node = Node(
        package='orbit_sim',
        executable='orbit_LVLH_gco',
        name='gco_controller',
        output='screen'
    )

    return LaunchDescription([
        start_world,
        odometry_bridge_node,
        imu_bridge_node,
        set_pose_bridge_node,
        wrench_bridge_node,
        gco_controller_node,
    ])
