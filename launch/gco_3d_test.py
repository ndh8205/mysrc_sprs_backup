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

    # 7. 3D PointCloud 브리지 - 올바른 토픽 사용
    lidar_3d_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_3d_bridge',
        arguments=[
            '/lidar/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # 8. IMU 브릿지 노드
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

    # 9. TF 변환 설정
    # world -> nasa_satellite3/nasa_satellite_link/lidar_3d (직접 연결)
    tf_world_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 
                   'world', 'nasa_satellite3/nasa_satellite_link/lidar_3d']
    )
    
    # 추가 TF (필요시) - base_link와 odom frame 연결
    tf_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )
    
    tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # 10. 3D SLAM 노드 - RTAB-Map ICP Odometry
    slam_3d_icp_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'subscribe_scan_cloud': True,
            'scan_cloud_topic': '/lidar/points_raw/points',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/MaxTranslation': '3.0',
            'Icp/VoxelSize': '0.2',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'OdomF2M/ScanSubtractRadius': '0.2',
            'OdomF2M/ScanMaxSize': '15000',
        }],
        remappings=[
            ('scan_cloud', '/lidar/points_raw/points')
        ],
        output='screen'
    )

    # 11. RTAB-Map SLAM 노드
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'subscribe_depth': False,
            'subscribe_scan_cloud': True,
            'scan_cloud_topic': '/lidar/points_raw/points',
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'use_sim_time': True,
            'queue_size': 10,
            'Grid/3D': 'true',
            'Grid/FromDepth': 'false',
            'Grid/RangeMax': '30',
            'Grid/ClusterRadius': '1',
            'Grid/GroundIsObstacle': 'true',
            'Grid/CellSize': '0.1',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '10',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Rtabmap/DetectionRate': '1',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '30',
            'Reg/Strategy': '1',  # ICP
            'Icp/MaxCorrespondenceDistance': '1.5',
            'Icp/Iterations': '30',
            'Icp/VoxelSize': '0.2',
        }],
        remappings=[
            ('scan_cloud', '/lidar/points_raw/points')
        ],
        output='screen'
    )

    # 12. 2D SLAM 옵션 (slam_toolbox) - 비활성화 상태
    # 3D 포인트클라우드를 2D LaserScan으로 변환이 필요하면 사용
    # pointcloud_to_laserscan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan',
    #     parameters=[{
    #         'target_frame': 'nasa_satellite3/nasa_satellite_link/lidar_3d',
    #         'min_height': -0.5,
    #         'max_height': 0.5,
    #         'angle_min': -3.14159,
    #         'angle_max': 3.14159,
    #         'angle_increment': 0.00436,
    #         'scan_time': 0.1,
    #         'range_min': 0.3,
    #         'range_max': 50.0,
    #         'use_inf': True,
    #     }],
    #     remappings=[
    #         ('cloud_in', '/lidar/points_raw/points'),
    #         ('scan', '/scan_2d')
    #     ]
    # )
    
    # slam_2d_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'solver_plugin': 'solver_plugins::CeresSolver',
    #         'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
    #         'ceres_preconditioner': 'SCHUR_JACOBI',
    #         'scan_topic': '/scan_2d',
    #     }],
    #     output='screen'
    # )
    
    return LaunchDescription([
        # Launch 파라미터
        csv_file1_arg,
        csv_file2_arg,
        csv_file3_arg,
        update_rate_arg,
        loop_data_arg,
        csv_data_rate_arg,
        time_scale_arg,
        
        # 기본 노드들
        start_world,
        camera_bridge_node,
        entity_bridge_node,
        satellite_controller_node,
        web_video_server_node,
        rosbridge_server_node,
        imu_bridge_node,

        # 3D 라이다 관련
        lidar_3d_bridge_node,  # 3D 포인트클라우드 브리지
        
        # TF 설정
        tf_world_to_lidar,
        tf_world_to_odom,
        tf_odom_to_base,
        
        # 3D SLAM (RTAB-Map 사용시)
        # slam_3d_icp_node,  # 필요시 활성화
        # rtabmap_slam_node,  # 필요시 활성화
        
        # 2D SLAM (slam_toolbox 사용시)
        # pointcloud_to_laserscan_node,  # 필요시 활성화
        # slam_2d_node,  # 필요시 활성화
    ])