# orbit_sim Python 노드 개발 가이드

> 이 파일은 `orbit_sim/orbit_sim/` 디렉토리의 Python 노드 개발 시 참조할 가이드입니다.

---

## 좌표계 컨벤션

| Frame | Symbol | Definition |
|-------|--------|------------|
| ECI | I | Earth-Centered Inertial (J2000) |
| LVLH | L | Chief 중심, r(radial), t(in-track), h(cross-track) |
| Body | B | 위성 동체 좌표계 |
| Camera | C | 카메라 좌표계 |

## 쿼터니언

- **형식**: Scalar-first `[qw, qx, qy, qz]` (ROS 표준: `geometry_msgs/Quaternion`)
- **주의**: ROS 메시지는 `(x, y, z, w)` 순서이나, 내부 연산 시 scalar-first 유지
- **Space_SLAM (MATLAB)**: scalar-first `[qw; qx; qy; qz]` 열벡터 → 이식 시 순서 확인 필수

## 단위 체계

| 물리량 | orbit_sim (ROS/Gazebo) | Space_SLAM (MATLAB) |
|--------|------------------------|---------------------|
| 위치 | **m** | km |
| 속도 | **m/s** | km/s |
| 각도/각속도 | **rad** / **rad/s** | rad / rad/s |
| 시간 | **s** | s |
| 질량 | **kg** | kg |

> **이식 시 단위 변환 누락이 가장 흔한 실수!** km → m 변환 (×1000) 반드시 확인.

---

## 노드 목록

### 암 제어 (Arm Control)

| 노드 | 파일 | 역할 |
|------|------|------|
| `impedance_controller` | impedance_controller.py | 단일 KARI 암 태스크 공간 임피던스 제어 (자유 부유체) |
| `dual_arm_impedance_controller` | dual_arm_impedance_controller.py | 듀얼 KARI 암 임피던스 제어 (좌/우 독립) |
| `torque_publisher` | torque_publisher_node.py | 토크 명령 퍼블리셔 (1초간 100Hz, 테스트용) |
| `force_torque_controller` | force_torque_controller.py | GUI 기반 힘/토크 제어 (오도메트리/IMU 모니터링) |
| — | kari_arm_dynamics.py | 유틸리티 모듈: 순기구학, 자코비안, 질량행렬, 모터 역학 (단일 암) |
| — | kari_dual_arm_dynamics.py | 유틸리티 모듈: 듀얼 암 역학 |

### 위성 제어 / 편대비행 (Satellite Control)

| 노드 | 파일 | 역할 |
|------|------|------|
| `multi_satellite_controller` | multi_satellite_controller.py | CSV 궤적 데이터 기반 다중 위성 포즈 퍼블리셔 |
| `multi_satellite_controller_service` | multi_satellite_controller_service.py | CSV 기반 위성 제어 (SetEntityPose 서비스, time_scale 지원) |
| `multi_satellite_controller_guidence` | multi_satellite_controller_guidence.py | 유도 알고리즘 + 서비스 기반 제어 + 보간 |
| `lvlh_sim_node` | lvlh_sim_node.py | LVLH 상대운동 시뮬레이터 (Chief/Deputy, CSV 로드, 자세 적용) |
| `gco_controller` | gco_controller.py | PID 기반 GCO 궤적 제어 (CW 방정식, 2개 Deputy 위성) |
| `orbit_LVLH_gco` | orbit_LVLH_gco.py | GUI: GCO + 수동 제어 모드 전환, 실시간 위성 포지셔닝 |

### 비전 / 센서 (Vision & Sensors)

| 노드 | 파일 | 역할 |
|------|------|------|
| `aruco_simulator` | aruco_simulator.py | ArUco 마커 이미지를 카메라 프레임에 오버레이 |
| `aruco_detector` | aruco_detector.py | ArUco 검출 → 호모그래피 분해 → PnP 자세추정 |
| `gazebo_aruco_controller` | gazebo_aruco_controller.py | 검출된 ArUco 포즈 기반 위성 포즈 명령 (폐루프) |
| `pose_control_camtest` | pose_control_camtest.py | GUI: 수동 위성 위치/자세 제어 (프리셋: 33km, 22km, 10km 등) |
| `imu_visualizer` | imu_visualizer.py | 실시간 IMU 시각화 (쿼터니언, 오일러, 각속도) |

### 센서 브리지 (Bridges)

| 노드 | 파일 | 역할 |
|------|------|------|
| `lidar_bridge` | lidar_bridge.py | Gazebo 3D LiDAR → PointCloud2 재퍼블리셔 (frame_id 갱신) |
| `laser_to_pointcloud` | laser_to_pointcloud.py | LaserScan (1D) → PointCloud2 (3D) 변환 |
| `http_ros_bridge` | http_ros_bridge.py | HTTP 서버: MATLAB → ROS 2 통신 (POST /target_pose_L/R, GET /status) |

---

## 주요 ROS 2 토픽 레퍼런스

### 단일 KARI 암

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/effort_controller/commands` | `Float64MultiArray` | 7축 토크 명령 |
| `/joint_states` | `JointState` | 관절 상태 (position/velocity/effort) |

### 듀얼 KARI 암

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/target_pose_L` | `PoseStamped` | 왼팔 목표 위치 |
| `/target_pose_R` | `PoseStamped` | 오른팔 목표 위치 |
| `/effort_controller_L/commands` | `Float64MultiArray` | 왼팔 토크 명령 (7 joints) |
| `/effort_controller_R/commands` | `Float64MultiArray` | 오른팔 토크 명령 (7 joints) |

### 커스텀 서비스

| 서비스 | 정의 | 용도 |
|--------|------|------|
| `/switch_algorithm` | `SwitchAlgorithm.srv` | 제어 알고리즘 런타임 전환 (int32 algorithm_id → bool success) |

---

## 노드 개발 시 주의사항

- 새 노드 추가 시 `setup.py`의 `entry_points`에 등록 필수 → rebuild 필요
- import 패턴: `from orbit_sim.kari_arm_dynamics import ...` (패키지 내부 모듈)
- `kari_arm_dynamics.py`, `kari_dual_arm_dynamics.py`는 entry_point 없는 유틸리티 모듈
- GUI 노드는 `python3-tk` 의존성 필요 (Tkinter)
- 비전 노드는 `python3-opencv`, `cv_bridge` 의존성 필요

---

*최종 업데이트: 2026-02-23*
