from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ

def generate_launch_description():
    # CSV 파일 경로 설정 kl   
    src_path = os.path.expanduser('~/space_ros_ws/src/orbit_sim/data')
    default_csv_path1 = os.path.join(src_path, 'sat1_state.csv')
    default_csv_path2 = os.path.join(src_path, 'sat3_state.csv')
    default_csv_path3 = os.path.join(src_path, 'sat4_state.csv')

    # orbit_sim 패키지의 월드 및 모델 파일 경로
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'gco_test.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    # 환경 변수 설정
    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([ # **<-- Gazebo Sim의 리소스 경로 변수**
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }

    # Launch 파라미터 선언
    csv_file1_arg = DeclareLaunchArgument(
        'csv_file1',
        default_value=default_csv_path1,
        description='Path to the CSV file for satellite 1'
    )
    csv_file2_arg = DeclareLaunchArgument(
        'csv_file2',
        default_value=default_csv_path2,
        description='Path to the CSV file for satellite 2'
    )
    csv_file3_arg = DeclareLaunchArgument(
        'csv_file3',
        default_value=default_csv_path3,
        description='Path to the CSV file for satellite 3'
    )
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='60.0',
        description='Update rate for satellite positions in Hz'
    )
    loop_data_arg = DeclareLaunchArgument(
        'loop_data',
        default_value='True',
        description='Whether to loop CSV data when reaching the end'
    )
    csv_data_rate_arg = DeclareLaunchArgument(
        'csv_data_rate',
        default_value='60.0',
        description='Sampling rate of CSV data in Hz'
    )
    time_scale_arg = DeclareLaunchArgument(
        'time_scale',
        default_value='1.0',
        description='Simulation time scale factor'
    )

    # 1. Gazebo 시뮬레이션 시작
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env=env
    )

    # 2. 카메라 브릿지 노드
    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/nasa_satellite/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/nasa_satellite2/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/nasa_satellite3/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/stereo/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/stereo/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )

    # 3. 위성 상태 제어 서비스 브릿지 노드
    entity_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='entity_bridge',
        arguments=[
            '/world/space_world/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean'
        ],
        output='screen'
    )

    # 4. 위성 제어 서비스 노드 (CSV 데이터 기반)
    satellite_controller_node = Node(
        package='orbit_sim',
        executable='multi_satellite_controller_service',
        name='multi_satellite_controller_service',
        output='screen',
        parameters=[{
            'csv_file1': LaunchConfiguration('csv_file1'),
            'csv_file2': LaunchConfiguration('csv_file2'),
            'csv_file3': LaunchConfiguration('csv_file3'),
            'update_rate': LaunchConfiguration('update_rate'),
            'loop_data': LaunchConfiguration('loop_data'),
            'csv_data_rate': LaunchConfiguration('csv_data_rate'),
            'time_scale': LaunchConfiguration('time_scale')
        }]
    )

    # web_video_server 노드 추가
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server_node',
        output='screen'
    )

    # rosbridge 서버 노드 추가
    rosbridge_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'
        }],
        output='screen'
    )

    return LaunchDescription([
        csv_file1_arg,
        csv_file2_arg,
        csv_file3_arg,
        update_rate_arg,
        loop_data_arg,
        csv_data_rate_arg,
        time_scale_arg,
        start_world,
        camera_bridge_node,
        entity_bridge_node,
        satellite_controller_node,
        web_video_server_node,
        rosbridge_server_node,
    ])