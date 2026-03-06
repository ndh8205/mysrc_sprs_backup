#!/usr/bin/env python3
"""
CubeSat VSCMG 자세제어 시뮬레이션 런치 파일
- CMG 동역학: Gazebo DART 물리엔진 (physical joints, kinematic velocity)
- 제어: PD + VSCMG 할당 → gimbal/wheel velocity 명령
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ


def generate_launch_description():
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'cubesat_cmg.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path,
        ])
    }

    # 1. Gazebo simulation
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env,
    )

    # 2. IMU bridge (GZ -> ROS)
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/cubesat_cmg/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    # 3. Joint state bridge (GZ -> ROS)
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/cubesat_cmg/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
    )

    # 4. Gimbal velocity command bridges (ROS -> GZ)
    gimbal_cmd_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gimbal_cmd_bridge',
        arguments=[
            '/cubesat_cmg/cmg1_gimbal/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/cubesat_cmg/cmg2_gimbal/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    # 5. Wheel velocity command bridges (ROS -> GZ)
    wheel_cmd_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wheel_cmd_bridge',
        arguments=[
            '/cubesat_cmg/cmg1_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/cubesat_cmg/cmg2_wheel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    # 6. Commander node (GUI + PD/VSCMG controller)
    commander_node = Node(
        package='orbit_sim',
        executable='cubesat_cmg_commander',
        name='cubesat_cmg_commander',
        output='screen',
    )

    return LaunchDescription([
        start_world,
        imu_bridge,
        joint_state_bridge,
        gimbal_cmd_bridge,
        wheel_cmd_bridge,
        commander_node,
    ])
