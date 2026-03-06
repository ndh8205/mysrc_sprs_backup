# orbit_sim — Space ROS Orbital Simulation Package

ROS 2 Jazzy + Gazebo Harmonic 기반 우주 근접운용(RPO) 시뮬레이션 패키지.
듀얼암 도킹, 편대비행, LiDAR 3D 매핑, DVS 이벤트 카메라, CMG 자세제어, 화성 회전익기 등을 지원합니다.

**환경:** WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (DART), NVIDIA GPU (D3D12)

---

## 주요 기능

| 기능 | 설명 |
|------|------|
| **GCO 근접운용** | Clohessy-Wiltshire 상대운동 기반 다중 위성 제어 |
| **LiDAR 3D 매핑** | DragonEye/TriDAR급 LiDAR로 대상 위성 3D 포인트클라우드 누적 |
| **DVS 이벤트 카메라** | DAVIS346 스펙 이벤트 카메라 에뮬레이션 ([gz_dvs_plugin](https://github.com/ndh8205/gz_dvs_plugin)) |
| **듀얼암 도킹** | 7-DOF KARI 암 x2 임피던스 제어 기반 도킹 |
| **CMG 자세제어** | VSCMG 테스트베드, CubeSat CMG, 화성 회전익기 CMG |
| **화성 회전익기** | CMG 보강 헥사콥터 (IAC 2026 논문 연구) |
| **스테레오 비전** | 스테레오 카메라 깊이/디스패리티 분석 |
| **편대비행** | CSV 궤적 기반 다중 위성 편대비행 시뮬레이션 |

---

## 빠른 시작

### 1. 워크스페이스 구축

```bash
mkdir -p ~/space_ros_ws/src && cd ~/space_ros_ws/src
git clone https://github.com/ndh8205/mysrc_sprs_backup.git orbit_sim
git clone https://github.com/ndh8205/gz_dvs_plugin.git       # DVS 플러그인 (선택)
cd ~/space_ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 2. 환경 변수 (~/.bashrc)

```bash
source ~/space_ros_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/space_ros_ws/install/orbit_sim/share/orbit_sim/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

WSL2 GPU 렌더링:
```bash
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH
export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"
export GALLIUM_DRIVER=d3d12
```

---

## 시뮬레이션 실행

### GCO + LiDAR 3D 매핑 + DVS 이벤트 카메라

```bash
ros2 launch orbit_sim gco_lidar_mapping.launch.py
# DVS 이벤트 → ros2 topic echo /dvs/events
# Gazebo GUI에서 카메라 + DVS 영상 확인 가능
# RViz 활성화: rviz:=true
```

### GCO 센서 시각화 (Gazebo GUI 내장)

```bash
ros2 launch orbit_sim sensor_viz_gco.launch.py
```

### 듀얼암 도킹

```bash
ros2 launch orbit_sim docking_sim.launch.py
ros2 run orbit_sim dual_arm_impedance_controller  # 별도 터미널
```

### KARI 암

```bash
ros2 launch orbit_sim kari_arm.launch.py           # 단일 암
ros2 launch orbit_sim kari_dual_arm.launch.py      # 듀얼 암
ros2 run orbit_sim impedance_controller            # 별도 터미널
```

### LVLH 상대운동

```bash
ros2 launch orbit_sim lvlh_sim.launch.py           # GUI 포함
ros2 launch orbit_sim lvlh_sim_nogui.launch.py     # 헤드리스
```

### 편대비행 / 다중 위성

```bash
ros2 launch orbit_sim fm_fly.launch.py
ros2 launch orbit_sim gco_test.launch.py
ros2 launch orbit_sim gco_cam_test.launch.py
```

### CMG 테스트베드

```bash
ros2 launch orbit_sim cmg_testbed.launch.py        # 듀얼 VSCMG 테스트
ros2 launch orbit_sim cubesat_cmg.launch.py        # CubeSat CMG
```

### 화성 회전익기

```bash
ros2 launch orbit_sim mars_hexacopter_cmg_test.launch.py       # CMG 듀얼
ros2 launch orbit_sim mars_hexacopter_cmg500_test.launch.py    # CMG-500 (30k RPM)
ros2 launch orbit_sim mars_drone.launch.py                     # 기본 드론
```

### 스테레오 카메라

```bash
ros2 launch orbit_sim stereo_test.launch.py
```

### LiDAR 테스트

```bash
ros2 launch orbit_sim lidar_test.launch.py
ros2 launch orbit_sim lidar_mapping_test.launch.py
```

### X500 쿼드콥터

```bash
ros2 launch orbit_sim x500_test.launch.py
```

### 비전 / ArUco

```bash
ros2 launch orbit_sim pnp_test.launch.py
ros2 launch orbit_sim experiment.launch.py
```

### HTTP-ROS 브리지 (MATLAB 연동)

```bash
ros2 run orbit_sim http_ros_bridge
```

---

## 프로세스 정리

```bash
bash ~/kill_sim.sh
```

---

## 패키지 구조

```
orbit_sim/
├── config/                                # 설정 파일
│   ├── kari_arm_controllers.yaml          #   단일 암 컨트롤러
│   ├── kari_dual_arm_controllers.yaml     #   듀얼 암 컨트롤러
│   └── lidar_mapping.rviz                 #   LiDAR 매핑 RViz 설정
│
├── data/                                  # 궤도/실험 데이터 (CSV)
│   ├── sat1_state.csv                     #   위성 1 궤적 (GCO)
│   ├── sat3_state.csv                     #   위성 3 궤적
│   ├── sat4_state.csv                     #   위성 4 궤적
│   ├── orbit_data_*.csv                   #   궤도 데이터 (1Hz/10Hz/100Hz)
│   └── cmg_testbed_*.csv                  #   CMG 테스트 데이터
│
├── docs/                                  # 문서
│   ├── mars_hexacopter_design.md          #   화성 회전익기 설계 사양
│   └── stereo_test_guide.md               #   스테레오 카메라 가이드
│
├── launch/                                # Launch 파일 (28개)
│   ├── ── GCO / 근접운용 ──
│   ├── gco_test.launch.py                 #   GCO 기본 테스트
│   ├── gco_cam_test.launch.py             #   GCO + 카메라 비전
│   ├── gco_lidar_mapping.launch.py        #   GCO + LiDAR 매핑 + DVS
│   ├── sensor_viz_gco.launch.py           #   GCO 센서 시각화 (GUI 내장)
│   ├── ── LiDAR ──
│   ├── lidar_test.launch.py               #   LiDAR 기본 테스트
│   ├── lidar_mapping_test.launch.py       #   LiDAR 3D 매핑
│   ├── ── 로봇암 / 도킹 ──
│   ├── docking_sim.launch.py              #   듀얼암 도킹
│   ├── kari_arm.launch.py                 #   단일 KARI 암
│   ├── kari_dual_arm.launch.py            #   듀얼 KARI 암
│   ├── ── 위성 / 편대비행 ──
│   ├── lvlh_sim.launch.py                 #   LVLH 상대운동
│   ├── lvlh_sim_nogui.launch.py           #   LVLH (헤드리스)
│   ├── fm_fly.launch.py                   #   편대비행
│   ├── sat.launch.py ~ sat4.launch.py     #   위성 시나리오
│   ├── ── CMG ──
│   ├── cmg_testbed.launch.py              #   VSCMG 테스트베드
│   ├── cmg_testbed_test.launch.py         #   CMG 테스트 변형
│   ├── cubesat_cmg.launch.py              #   CubeSat CMG
│   ├── ── 화성 ──
│   ├── mars_drone.launch.py               #   화성 드론
│   ├── mars_hexacopter.launch.py          #   화성 헥사콥터
│   ├── mars_hexacopter_cmg_test.launch.py #   화성 헥사콥터 CMG
│   ├── mars_hexacopter_cmg500_test.launch.py # CMG-500 (30k RPM)
│   ├── ── 기타 ──
│   ├── stereo_test.launch.py              #   스테레오 카메라
│   ├── x500_test.launch.py                #   X500 쿼드콥터
│   ├── pnp_test.launch.py                 #   PnP 포즈 추정
│   └── experiment.launch.py               #   실험 시나리오
│
├── models/                                # Gazebo 모델 (51개)
│   ├── ── 위성 ──
│   ├── nasa_satellite ~ nasa_satellite6/  #   NASA 위성 시리즈
│   ├── gaim_target_3u/                    #   GAiMSat 3U CubeSat
│   ├── gaimsat_target/                    #   GAiMSat 타겟 (메시 모델)
│   ├── intel_sat_dummy/                   #   IntelSat 더미
│   ├── capsule/                           #   도킹 캡슐
│   ├── iss/                               #   ISS
│   ├── ── 로봇암 ──
│   ├── canadarm/                          #   Canadarm2
│   ├── kari_arm/                          #   KARI 단일 암 (7-DOF)
│   ├── kari_arm_only/                     #   KARI 독립 암
│   ├── kari_dual_arm/                     #   듀얼 KARI 암
│   ├── ── CMG ──
│   ├── cmg_testbed/                       #   듀얼 VSCMG 테스트베드
│   ├── cubesat_cmg/                       #   CubeSat CMG
│   ├── ── 화성 회전익기 ──
│   ├── mars_hexacopter/                   #   화성 헥사콥터
│   ├── mars_hexacopter_base/              #   베이스 변형
│   ├── mars_hexacopter_cmg500/            #   CMG-500 장착형
│   ├── mars_hexacopter_cmg500_base/       #   CMG-500 베이스
│   ├── ── 드론 / UAV ──
│   ├── x500/, x500_base/, x500_mars/     #   X500 쿼드콥터 시리즈
│   ├── ardupilot_hexapod_copter/          #   ArduPilot 헥사팟
│   ├── nasa_ingenuity/                    #   Ingenuity
│   ├── uam_1/, uam_3/                    #   UAM
│   ├── ── 지상 실험 ──
│   ├── airbearing_satellite/              #   에어베어링 테스트
│   ├── controla_prototype_1~2/            #   ControLA 프로토타입
│   ├── hagi/, hsr_1/                      #   HAGI, HSR
│   ├── aruco_marker/, hokuyo/             #   센서 / 마커
│   ├── ── 행성 / 환경 ──
│   ├── earth/, saturn/                    #   행성
│   ├── lunar_surface/, moon_base/         #   달
│   ├── martian_surface/                   #   화성
│   ├── enceladus_surface/, ocean_surface/ #   엔셀라두스
│   └── solar_panel/, truss/, submarine/   #   기타
│
├── orbit_sim/                             # Python 노드 (26개 entry_points)
│   ├── ── 위성 제어 ──
│   ├── multi_satellite_controller.py      #   다중 위성 CSV 궤적 제어
│   ├── multi_satellite_controller_service.py # 서비스 기반 포즈 제어 + TF
│   ├── multi_satellite_controller_guidence.py # 유도 알고리즘
│   ├── gco_controller.py                  #   GCO PID 제어
│   ├── orbit_LVLH_gco.py                 #   LVLH GCO GUI
│   ├── lvlh_sim_node.py                   #   LVLH 시뮬레이터
│   ├── ── 암 제어 ──
│   ├── impedance_controller.py            #   단일 암 임피던스
│   ├── dual_arm_impedance_controller.py   #   듀얼 암 임피던스
│   ├── force_torque_controller.py         #   힘/토크 GUI 제어
│   ├── torque_publisher_node.py           #   토크 퍼블리셔
│   ├── ── 센서 / 비전 ──
│   ├── aruco_detector.py                  #   ArUco 검출 + PnP
│   ├── aruco_simulator.py                 #   ArUco 시뮬레이터
│   ├── gazebo_aruco_controller.py         #   ArUco 기반 포즈 제어
│   ├── imu_visualizer.py                  #   IMU 시각화
│   ├── stereo_test_gui.py                 #   스테레오 깊이 분석 GUI
│   ├── pose_control_camtest.py            #   카메라 포즈 GUI
│   ├── ── LiDAR ──
│   ├── lidar_bridge.py                    #   LiDAR 브리지
│   ├── laser_to_pointcloud.py             #   레이저→포인트클라우드
│   ├── lidar_orbit_scanner.py             #   궤도 스캐너
│   ├── pointcloud_mapper.py              #   3D 포인트클라우드 누적 매퍼
│   ├── ── CMG ──
│   ├── cmg_testbed_commander.py           #   CMG 테스트베드 GUI
│   ├── cmg_testbed_scenario.py            #   CMG 시나리오 실행
│   ├── cubesat_cmg_commander.py           #   CubeSat CMG GUI
│   ├── cubesat_cmg_controller.py          #   CubeSat CMG 제어
│   ├── mars_hexacopter_cmg_commander.py   #   화성 헥사콥터 CMG GUI
│   └── http_ros_bridge.py                #   HTTP↔ROS 브리지
│
├── scripts/                               # 테스트 / 유틸리티
├── srv/SwitchAlgorithm.srv                # 알고리즘 전환 서비스
├── urdf/                                  # URDF (KARI 암)
├── worlds/                                # Gazebo World 파일 (31개)
├── setup.py
├── setup.cfg
└── package.xml
```

---

## 관련 패키지

| 패키지 | 설명 |
|--------|------|
| [gz_dvs_plugin](https://github.com/ndh8205/gz_dvs_plugin) | DVS 이벤트 카메라 Gazebo 플러그인 (DAVIS346 에뮬레이션) |
| [Space_SLAM](https://github.com/ndh8205) | MATLAB 기반 항법 필터 / SLAM 알고리즘 연구 |

---

## 의존성

### 시스템
- Ubuntu 24.04 (WSL2)
- ROS 2 Jazzy Desktop
- Gazebo Harmonic
- NVIDIA GPU + D3D12 렌더링

### ROS 패키지
```
ros-jazzy-ros-gz  ros-jazzy-gz-ros2-control  ros-jazzy-moveit
ros-jazzy-ros2-control  ros-jazzy-ros2-controllers
ros-jazzy-joint-state-publisher  ros-jazzy-xacro
ros-jazzy-robot-state-publisher  ros-jazzy-controller-manager
ros-jazzy-slam-toolbox  ros-jazzy-navigation2  ros-jazzy-cv-bridge
```

### Python
```
numpy  scipy  matplotlib  opencv-python  tkinter
```

---

## 주요 ROS 2 토픽

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/dvs/events` | `gz_dvs_plugin/msg/DvsEventArray` | DVS 이벤트 스트림 |
| `/lidar/points_raw/points` | `sensor_msgs/PointCloud2` | LiDAR 포인트클라우드 |
| `/pointcloud_map` | `sensor_msgs/PointCloud2` | 누적 3D 맵 |
| `/target_pose_L`, `_R` | `geometry_msgs/PoseStamped` | 암 목표 위치 |
| `/effort_controller_L/commands` | `Float64MultiArray` | 암 토크 명령 |
| `/joint_states` | `sensor_msgs/JointState` | 관절 상태 |

---

## 트러블슈팅

| 증상 | 해결 |
|------|------|
| GPU 소프트웨어 렌더링 | `GALLIUM_DRIVER=d3d12` 확인 |
| 모델 로딩 실패 | `GZ_SIM_RESOURCE_PATH` 확인 |
| DVS 플러그인 로드 실패 | `GZ_SIM_SYSTEM_PLUGIN_PATH`에 gz_dvs_plugin lib 경로 추가 |
| 관절 떨림/폭발 | `<self_collide>false` + `<disable_collisions>` 설정 |
| 프로세스 정리 안 됨 | `bash ~/kill_sim.sh` |

---

*최종 업데이트: 2026-03-06*
