from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ

def generate_launch_description():
    # 패키지 경로
    orbit_sim_path = get_package_share_directory('orbit_sim')

    model_path = os.path.join(orbit_sim_path, 'models') # **<-- 모델 경로를 명시적으로 지정**
    
    # 월드 파일
    world_file = os.path.join(orbit_sim_path, 'worlds', 'artag_test_inc_model.world')
    
    # 환경 변수 설정
    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([ # **<-- Gazebo Sim의 리소스 경로 변수**
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }
    
    # 런치 파라미터 선언
    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='0',
        description='ArUco 마커 ID')
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.8',
        description='마커 크기 (미터)')
    
    # Gazebo 시뮬레이션 시작
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        additional_env=env
    )
    
    # 서비스 브릿지: Ignition Gazebo의 set_pose 서비스를 ROS 2로 브리지
    bridge_set_pose = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/world/space_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        output='screen'
    )
    # 카메라 토픽 브릿지: Gazebo의 카메라 토픽 ↔ ROS 2 이미지 메시지
    bridge_camera = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/nasa_satellite2/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )
    
    # 아루코 시뮬레이터 노드
    aruco_simulator_node = Node(
        package='orbit_sim',
        executable='aruco_simulator',
        name='aruco_simulator',
        parameters=[{
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size': LaunchConfiguration('marker_size')
        }],
        output='screen'
    )
    
    # 아루코 감지기 노드 (추가됨)
    aruco_detector_node = Node(
        package='orbit_sim',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[{
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size': LaunchConfiguration('marker_size')
        }],
        output='screen'
    )


    # web_video_server 노드 추가
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server_node',
        parameters=[{'port': 8080}]  # 8080 포트를 사용하도록 설정
    )
    

    return LaunchDescription([
        marker_id_arg,
        marker_size_arg,
        start_world,
        bridge_set_pose,
        bridge_camera,
        aruco_simulator_node,
        aruco_detector_node,
        web_video_server_node     
    ])