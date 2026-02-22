from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from os import environ

def generate_launch_description():
    # CSV 파일 경로 설정
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
        'GZ_SIM_RESOURCE_PATH': ':'.join([
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

    # 5. Web Video Server 노드
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server_node',
        output='screen'
    )

    # 6. ROS Bridge Server 노드
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

    # 기존 브릿지 (전체 경로 사용)
    lidar_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_gz_bridge',
        arguments=[
            '/world/space_world/model/nasa_satellite3/link/nasa_satellite_link/sensor/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # 커스텀 브릿지 노드 추가
    lidar_custom_bridge_node = Node(
        package='orbit_sim',
        executable='lidar_bridge',
        name='lidar_custom_bridge',
        output='screen'
    )

    # PointCloud2 직접 브릿지
    #lidar_bridge_node = Node(
    #    package='ros_gz_bridge',
    #    executable='parameter_bridge',
    #    name='lidar_bridge',
    #    arguments=[
    #        'lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
    #    ],
    #    output='screen'
    #)
    # PointCloudPacked 직접 브리지 시도
    lidar_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_gz_bridge',
        arguments=[
            '/lidar/points_raw@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # 8. IMU 브릿지 노드 (선택사항)
    imu_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/nasa_satellite/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/nasa_satellite2/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/nasa_satellite3/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/nasa_satellite4/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        output='screen'
    )

    laser_to_pointcloud_node = Node(
        package='orbit_sim',
        executable='laser_to_pointcloud',
        name='laser_to_pointcloud',
        output='screen'
    )

    tf_world_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'nasa_satellite3/nasa_satellite_link/lidar_3d']
    )    

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': True,
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'scan_topic': '/lidar/points',
        }],
        output='screen'
    )    

    
    return LaunchDescription([
        # Launch 파라미터
        csv_file1_arg,
        csv_file2_arg,
        csv_file3_arg,
        update_rate_arg,
        loop_data_arg,
        csv_data_rate_arg,
        time_scale_arg,
        
        # 노드들
        start_world,
        camera_bridge_node,
        entity_bridge_node,
        satellite_controller_node,
        web_video_server_node,
        rosbridge_server_node,
        imu_bridge_node,

        # 라이다 브릿지 
        #lidar_bridge_node,  # LaserScan 타입만 사용
        lidar_gz_bridge_node,
        tf_world_to_lidar,
        slam_node,
    ])