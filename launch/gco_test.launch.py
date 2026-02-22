#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ

def generate_launch_description():
    # orbit_sim 패키지의 월드 및 모델 파일 경로
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'orbit_LVLH_GCO.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    # 환경 변수 설정
    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }

    # 1. Gazebo 시뮬레이션 시작
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # 2. Odometry 브릿지 노드 (위치/속도 측정용)
    odometry_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odometry_bridge',
        arguments=[
            '/model/nasa_satellite5/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )

    # 3. IMU 브릿지 노드 (자세/각속도 측정용)
    imu_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/nasa_satellite5/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    set_pose_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='set_pose_bridge',
        arguments=[
            '/world/space_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        output='screen'
    )

    # 4. 카메라 브릿지 노드 (시각적 확인용)
    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/nasa_satellite5/camera@sensor_msgs/msg/Image@gz.msgs.Image',

            # VSS NFOV 스테레오 페어
            '/mev/vss_nfov/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_nfov/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            
            # VSS WFOV 스테레오 페어  
            '/mev/vss_wfov/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_wfov/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            
            # VSS 도킹 카메라 스테레오 페어
            '/mev/vss_docking/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_docking/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen'
    )

    wrench_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wrench_bridge',
        arguments=[
            '/world/space_world/wrench@ros_gz_interfaces/msg/EntityWrench@gz.msgs.EntityWrench'
        ],
        output='screen'
    )

    # 5. Web Video Server (선택사항 - 웹에서 카메라 영상 보기용)
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server_node',
        output='screen'
    )

    gco_controller_node = Node(
        package='orbit_sim',
        executable='orbit_LVLH_gco',  # 새로운 실행 파일명
        name='gco_controller',
        output='screen'
    )

    return LaunchDescription([
        start_world,
        odometry_bridge_node,
        imu_bridge_node,
        camera_bridge_node,
        web_video_server_node,
        gco_controller_node,
        wrench_bridge_node,
        set_pose_bridge_node,
    ])