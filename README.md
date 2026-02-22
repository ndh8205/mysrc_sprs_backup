# 🛰️ WSL2 Ubuntu 24.04에서 위성 근접운용 시뮬레이션 재구축 가이드

**환경:** Windows 10/11 (WSL2), Ubuntu 24.04, NVIDIA GPU  
**목표:** Docker 없이 ROS 2 Jazzy + Gazebo Harmonic 환경에서 구축  

---

## 📌 목차
1. [WSL2 환경 설정](#1-wsl2-환경-설정)
2. [ROS 2 Jazzy 설치](#2-ros-2-jazzy-설치)
3. [Gazebo Harmonic 설치](#3-gazebo-harmonic-설치)
4. [ROS-Gazebo 연동 및 추가 패키지 설치](#4-ros-gazebo-연동-및-추가-패키지-설치)
5. [WSL2 GPU 렌더링 설정](#5-wsl2-gpu-렌더링-설정)
6. [워크스페이스 구축 및 빌드](#6-워크스페이스-구축-및-빌드)
7. [환경 변수 설정](#7-환경-변수-설정)
8. [시뮬레이션 실행](#8-시뮬레이션-실행)
9. [orbit_sim 패키지 구조](#9-orbit_sim-패키지-구조)
10. [주요 ROS 2 토픽 및 명령어 레퍼런스](#10-주요-ros-2-토픽-및-명령어-레퍼런스)
11. [Gazebo 모델 설정 주요 사항](#11-gazebo-모델-설정-주요-사항)
12. [트러블슈팅](#12-트러블슈팅)
13. [향후 확장 방향](#13-향후-확장-방향)

---

## 1. WSL2 환경 설정

### 1.1 Ubuntu 24.04 설치
```powershell
wsl --install -d Ubuntu-24.04
```

### 1.2 WSL2 자원 제한 (선택 권장)
`C:\Users\<사용자명>\.wslconfig` 파일 작성:
```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
```
변경 후:
```cmd
wsl --shutdown
```
초기화가 필요한 경우:
```bash
wsl --terminate Ubuntu-24.04
wsl --unregister Ubuntu-24.04
wsl --install -d Ubuntu-24.04
```
> **⚠️ 주의:** `--unregister`는 배포판 **완전 삭제** (내부 데이터 복구 불가). `--terminate`는 실행 중인 인스턴스 강제 종료.

재부팅 후 Ubuntu 사용자명 및 암호를 설정합니다.

### 1.3 로케일 및 기본 도구 설치
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales curl gnupg2 lsb-release wget git-lfs mesa-utils libasio-dev
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

---

## 2. ROS 2 Jazzy 설치

### 2.1 저장소 키 및 저장소 추가
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.2 ROS 2 Jazzy 설치
```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 2.3 추가 개발 도구
```bash
sudo apt install -y python3-pip python3-colcon-mixin python3-flake8 python3-pytest-cov \
  python3-rosinstall-generator ros-jazzy-ament-* ros-jazzy-ros-testing ros-jazzy-eigen3-cmake-module
```

### 2.4 ROS 2 환경 초기화
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init || true
rosdep update

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
```

### 2.5 설치 확인
```bash
ros2 --help
printenv ROS_DISTRO  # "jazzy" 출력 확인
```

---

## 3. Gazebo Harmonic 설치

### 3.1 OSRF 저장소 추가
```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt update
```

### 3.2 Gazebo Harmonic 설치
```bash
sudo apt install -y gz-harmonic
```

설치 확인:
```bash
gz sim --version
```

---

## 4. ROS-Gazebo 연동 및 추가 패키지 설치

### 4.1 핵심 연동 패키지
```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-controller-manager
```

### 4.2 추가 기능 패키지
```bash
sudo apt install -y \
  ros-jazzy-rosbridge-server \
  ros-jazzy-web-video-server \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-cv-bridge
```

### 4.3 Python 의존성
orbit_sim 노드들이 사용하는 Python 패키지:
```bash
sudo apt install -y \
  python3-numpy \
  python3-scipy \
  python3-matplotlib \
  python3-opencv \
  python3-tk
```

| 패키지 | 사용처 |
|--------|--------|
| `numpy`, `scipy` | imu_visualizer (쿼터니언 변환, Rotation) |
| `matplotlib` | imu_visualizer (실시간 플롯) |
| `python3-tk` | pose_control_camtest (Tkinter GUI) |
| `python3-opencv` | aruco_detector, aruco_simulator, gazebo_aruco_controller |

### 4.4 GPU 사용자 그룹 추가
```bash
sudo usermod -aG render $USER
```
> **참고:** 재부팅 후 적용됩니다.

---

## 5. WSL2 GPU 렌더링 설정

> **핵심:** WSLg 환경에서는 `DISPLAY=:0`이 자동 설정되므로 별도 지정 불필요.  
> `LIBGL_ALWAYS_INDIRECT`, `__NV_PRIME_RENDER_OFFLOAD`, `__GLX_VENDOR_LIBRARY_NAME`, `MESA_GL_VERSION_OVERRIDE` 등은 불필요 — 아래 3개 변수만으로 충분.

### 5.1 환경 변수 설정
```bash
cat >> ~/.bashrc << 'EOF'

# === WSL2 GPU Rendering ===
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH
export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"
export GALLIUM_DRIVER=d3d12
EOF

source ~/.bashrc
```

### 5.2 렌더링 확인
```bash
glxinfo | grep "OpenGL renderer"
```
기대 출력:
```
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX ...)
```

### 5.3 렌더링 안 될 경우
```bash
sudo apt install --reinstall libegl-mesa0 libgl1-mesa-dri
```

> **주의:** `.bashrc`에 동일 환경변수를 중복 추가하지 않도록 주의.  
> 중복 여부 확인: `grep "GALLIUM_DRIVER" ~/.bashrc`  
> 중복 정리: `sed -i '/GALLIUM_DRIVER/d' ~/.bashrc` 후 재설정

---

## 6. 워크스페이스 구축 및 빌드

### 6.1 워크스페이스 생성 및 orbit_sim 클론
```bash
mkdir -p ~/space_ros_ws/src
cd ~/space_ros_ws/src

git clone https://github.com/ndh8205/mysrc_sprs_backup.git orbit_sim
```

### 6.2 의존성 설치
```bash
cd ~/space_ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 6.3 빌드
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> orbit_sim만 있으므로 `--allow-overriding`, `--skip-keys` 불필요.

> **클린 빌드가 필요한 경우:**
> ```bash
> rm -rf build/ install/ log/
> colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
> ```

### 6.4 orbit_sim만 단독 빌드 (수정 후 빠른 반복)
```bash
cd ~/space_ros_ws
colcon build --symlink-install --packages-select orbit_sim
source install/setup.bash
```

---

## 7. 환경 변수 설정

### 7.1 필수 환경 변수
```bash
cat >> ~/.bashrc << 'EOF'

# === Space ROS Workspace ===
source ~/space_ros_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/space_ros_ws/install/orbit_sim/share/orbit_sim/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
EOF

source ~/.bashrc
```

### 7.2 환경 변수 확인
```bash
echo $GZ_SIM_RESOURCE_PATH
# 출력에 orbit_sim/models 경로가 포함되어 있어야 함

echo $GZ_SIM_SYSTEM_PLUGIN_PATH
# gz_ros2_control 플러그인 경로가 포함되어 있어야 함
```

---

## 8. 시뮬레이션 실행

### 8.1 듀얼암 도킹 시뮬레이션
```bash
cd ~/space_ros_ws
source install/setup.bash
ros2 launch orbit_sim docking_sim.launch.py
```
별도 터미널에서 임피던스 컨트롤러 실행:
```bash
source ~/space_ros_ws/install/setup.bash
ros2 run orbit_sim dual_arm_impedance_controller
```

### 8.2 KARI 암 시뮬레이션
```bash
# 단일 KARI 암
ros2 launch orbit_sim kari_arm.launch.py

# 듀얼 KARI 암
ros2 launch orbit_sim kari_dual_arm.launch.py
```
별도 터미널에서:
```bash
ros2 run orbit_sim impedance_controller          # 단일 암
ros2 run orbit_sim dual_arm_impedance_controller  # 듀얼 암
```

### 8.3 LVLH 상대운동 시뮬레이션
```bash
ros2 launch orbit_sim lvlh_sim.launch.py          # GUI 포함
ros2 launch orbit_sim lvlh_sim_nogui.launch.py    # GUI 없이 (성능 우선)
```

### 8.4 편대비행 / 다중 위성
```bash
ros2 launch orbit_sim fm_fly.launch.py            # 편대비행
ros2 launch orbit_sim sat.launch.py               # 위성 기본
ros2 launch orbit_sim sat2.launch.py              # Canadarm 연동
ros2 launch orbit_sim sat3.launch.py              # 위성 3
ros2 launch orbit_sim sat4.launch.py              # 위성 4
```

### 8.5 GCO 제어 테스트
```bash
ros2 launch orbit_sim gco_test.launch.py          # GCO 제어 테스트
ros2 launch orbit_sim gco_cam_test.launch.py      # GCO 카메라 테스트
```

### 8.6 비전/ArUco 테스트
```bash
ros2 launch orbit_sim pnp_test.launch.py          # PnP 포즈 추정 테스트
ros2 launch orbit_sim experiment.launch.py        # 실험 시나리오
```

### 8.7 HTTP-ROS 브리지 (웹 인터페이스 연동)
```bash
ros2 run orbit_sim http_ros_bridge
```
WSL IP 확인:
```bash
hostname -I
```

### 8.8 알고리즘 전환 (서비스 호출)
```bash
# SwitchAlgorithm 서비스로 제어 알고리즘 런타임 전환
ros2 service call /switch_algorithm orbit_sim/srv/SwitchAlgorithm "{algorithm: 'new_algo'}"
```

---

## 9. orbit_sim 패키지 구조

> src에는 `orbit_sim`만 존재. 초기 구축 시 함께 클론했던 `demos`, `simulation`, `qt_gui_core`, `ros_gz`, `vision_msgs`, `gps_msgs`는 불필요하여 제거함.
```
orbit_sim/
├── config/                              # 컨트롤러 설정
│   ├── kari_arm_controllers.yaml
│   └── kari_dual_arm_controllers.yaml
│
├── data/                                # 궤도 데이터 (CSV)
│   ├── orbit_data.csv
│   ├── orbit_data_100hz.csv
│   ├── orbit_data_10hz.csv
│   ├── orbit_data_1hz.csv
│   ├── sat1_state.csv
│   ├── sat3_state.csv
│   └── sat4_state.csv
│
├── launch/                              # Launch 파일
│   ├── docking_sim.launch.py            #   도킹 시뮬레이션
│   ├── kari_arm.launch.py               #   단일 KARI 암
│   ├── kari_dual_arm.launch.py          #   듀얼 KARI 암
│   ├── lvlh_sim.launch.py              #   LVLH 상대운동 시뮬레이션
│   ├── lvlh_sim_nogui.launch.py        #   LVLH (GUI 없이)
│   ├── fm_fly.launch.py                #   편대비행 시뮬레이션
│   ├── experiment.launch.py            #   실험 시나리오
│   ├── gco_test.launch.py             #   GCO 제어 테스트
│   ├── gco_cam_test.launch.py         #   GCO 카메라 테스트
│   ├── gco_3d_test.py                 #   GCO 3D 테스트
│   ├── pnp_test.launch.py            #   PnP 포즈 추정 테스트
│   ├── sat.launch.py                  #   위성 기본
│   ├── sat2.launch.py                 #   위성 2 (Canadarm)
│   ├── sat3.launch.py                 #   위성 3
│   └── sat4.launch.py                 #   위성 4
│
├── models/                              # Gazebo 모델 (37개)
│   ├── ── 위성/우주 ──
│   ├── nasa_satellite/                  #   NASA 위성 기본
│   ├── nasa_satellite2~6/               #   NASA 위성 변형 (편대비행용)
│   ├── gaim_target_3u/                  #   GAiMSat Target 3U 큐브샛
│   ├── intel_sat_dummy/                 #   Target 위성 더미
│   ├── capsule/                         #   도킹 캡슐
│   ├── iss/                             #   ISS 모델
│   ├── earth/                           #   지구 모델
│   ├── saturn/                          #   토성 모델
│   ├── ── 로봇암/위성본체 ──
│   ├── canadarm/                        #   Canadarm2 (Space ROS)
│   ├── kari_arm/                        #   KARI 단일 암
│   ├── kari_arm_only/                   #   KARI 독립 암
│   ├── kari_dual_arm/                   #   듀얼 암 + Chaser 위성 본체
│   ├── ── 지상 실험 ──
│   ├── airbearing_satellite/            #   에어베어링 테스트베드 위성
│   ├── controla_prototype_1/            #   ControLA 프로토타입 1
│   ├── controla_prototype_2/            #   ControLA 프로토타입 2
│   ├── hagi/                            #   HAGI 모델
│   ├── hsr_1/                           #   HSR 모델
│   ├── aruco_marker/                    #   ArUco 마커
│   ├── hokuyo/                          #   Hokuyo LiDAR
│   ├── ── UAM ──
│   ├── uam_1/                           #   UAM 모델 1
│   ├── uam_3/                           #   UAM 모델 3
│   ├── ── 행성/환경 ──
│   ├── lunar_surface/                   #   달 표면
│   ├── martian_surface/                 #   화성 표면
│   ├── moon_base/                       #   달 기지
│   ├── ocean_surface/                   #   해양 표면 (Enceladus)
│   ├── enceladus_surface/               #   엔셀라두스 표면
│   ├── ── 기타 ──
│   ├── nasa_ingenuity/                  #   Ingenuity 헬리콥터
│   ├── nasa_perseverance/               #   Perseverance 로버
│   ├── submarine/                       #   잠수함 (Enceladus)
│   ├── solar_panel/                     #   태양전지판
│   ├── truss/                           #   트러스 구조물
│   ├── curiosity_path/                  #   Curiosity 경로
│   ├── X1/                              #   X1 로버
│   └── X2/                              #   X2 로버
│
├── orbit_sim/                           # Python 노드 (21개)
│   ├── __init__.py
│   ├── ── 암 제어 ──
│   ├── impedance_controller.py          #   단일 암 임피던스 제어
│   ├── dual_arm_impedance_controller.py #   듀얼 암 임피던스 제어
│   ├── kari_arm_dynamics.py             #   단일 암 역학
│   ├── kari_dual_arm_dynamics.py        #   듀얼 암 역학
│   ├── force_torque_controller.py       #   힘/토크 제어
│   ├── torque_publisher_node.py         #   토크 퍼블리셔
│   ├── ── 위성 제어/편대비행 ──
│   ├── multi_satellite_controller.py    #   다중 위성 제어
│   ├── multi_satellite_controller_guidence.py  # 유도 알고리즘
│   ├── multi_satellite_controller_service.py   # 서비스 인터페이스
│   ├── lvlh_sim_node.py                 #   LVLH 상대운동 시뮬레이션
│   ├── orbit_LVLH_gco.py               #   궤도 LVLH GCO 제어
│   ├── ── 비전/센서 ──
│   ├── aruco_detector.py                #   ArUco 마커 검출
│   ├── aruco_simulator.py               #   ArUco 시뮬레이터
│   ├── gazebo_aruco_controller.py       #   Gazebo ArUco 연동 제어
│   ├── gco_controller.py                #   GCO 제어기
│   ├── pose_control_camtest.py          #   카메라 포즈 제어 테스트
│   ├── imu_visualizer.py                #   IMU 시각화
│   ├── ── 센서 브리지 ──
│   ├── laser_to_pointcloud.py           #   레이저 → 포인트클라우드 변환
│   ├── lidar_bridge.py                  #   LiDAR 브리지
│   └── http_ros_bridge.py               #   HTTP ↔ ROS 브리지
│
├── scripts/                             # 테스트/유틸리티
│   ├── ros2_gz_test.py
│   ├── data_logger.py
│   ├── gco_test.py
│   ├── lvlh_sim_node_backup.py
│   └── test.py
│
├── srv/                                 # 커스텀 서비스
│   └── SwitchAlgorithm.srv              #   알고리즘 전환 서비스
│
├── urdf/                                # URDF 파일
│   ├── kari_arm.urdf
│   └── kari_dual_arm.urdf
│
├── worlds/                              # Gazebo World 파일 (20개)
│   ├── ── 궤도 환경 ──
│   ├── orbit.sdf                        #   기본 궤도 (무중력)
│   ├── orbit2.world                     #   궤도 변형
│   ├── orbit_GEO.world                  #   GEO 궤도
│   ├── orbit_LVLH_GCO.world            #   LVLH 상대운동 + GCO
│   ├── docking_world.sdf               #   도킹 시뮬레이션
│   ├── ── 로봇암 ──
│   ├── kari_arm.sdf                     #   단일 KARI 암
│   ├── kari_dual_arm.sdf               #   듀얼 KARI 암
│   ├── ── 지상 실험 ──
│   ├── airbearing_testbed.world        #   에어베어링 테스트베드
│   ├── experiment.world                #   실험 환경
│   ├── gco_test.world                  #   GCO 테스트
│   ├── gco_test_VICON.world            #   GCO + VICON
│   ├── gco_test_control.world          #   GCO 제어 테스트
│   ├── gco_controla.world              #   GCO ControLA
│   ├── gco_fm_VICON.world              #   GCO 편대비행 + VICON
│   ├── artag_test_inc_model.world      #   AR태그 테스트
│   ├── ── 행성 환경 (spaceros_gz_demos 기반) ──
│   ├── moon.sdf                        #   달 표면
│   ├── mars.sdf                        #   화성 표면
│   ├── enceladus.sdf                   #   엔셀라두스
│   ├── simple.world                    #   기본 world
│   └── sat.world                       #   위성 기본
│
├── setup.py
├── setup.cfg
├── package.xml
└── resource/orbit_sim
```

---

## 10. 주요 ROS 2 토픽 및 명령어 레퍼런스

### 10.1 듀얼암 도킹 시뮬레이션 토픽

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/target_pose_L` | `geometry_msgs/PoseStamped` | 왼팔 목표 위치 |
| `/target_pose_R` | `geometry_msgs/PoseStamped` | 오른팔 목표 위치 |
| `/effort_controller_L/commands` | `std_msgs/Float64MultiArray` | 왼팔 토크 명령 (7 joints) |
| `/effort_controller_R/commands` | `std_msgs/Float64MultiArray` | 오른팔 토크 명령 (7 joints) |
| `/joint_states` | `sensor_msgs/JointState` | 전체 관절 상태 |

### 10.2 Gazebo 네이티브 토픽 (gz topic)

| 토픽 | 용도 |
|------|------|
| `/world/kari_arm_world/model/kari_arm/joint_state` | Gazebo 관절 상태 |
| `/world/kari_arm_world/pose/info` | 모델 pose 정보 |
| `/model/kari_arm/joint/joint1/cmd_force` | 관절 토크 직접 인가 |

### 10.3 자주 사용하는 명령어

**목표 위치 퍼블리시 (듀얼 암):**
```bash
# 왼팔
ros2 topic pub /target_pose_L geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 2.5, y: 0.13, z: 0.0}, \
  orientation: {w: 0.7071, x: -0.7071, y: 0.0, z: 0.0}}}" --once

# 오른팔
ros2 topic pub /target_pose_R geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 2.5, y: -0.13, z: 0.0}, \
  orientation: {w: 0.7071, x: 0.7071, y: 0.0, z: 0.0}}}" --once
```

**직접 토크 인가 (단일 암):**
```bash
ros2 topic pub --once /effort_controller_L/commands \
  std_msgs/msg/Float64MultiArray "{data: [5.0, 0, 0, 0, 0, 0, 0]}"
```

**상태 확인:**
```bash
ros2 topic list | grep joint
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 node list | grep impedance
```

**Gazebo 토크 직접 인가 (ros_gz 브리지 우회):**
```bash
gz topic -t "/model/kari_arm/joint/joint1/cmd_force" \
  -m gz.msgs.Double -p "data: 10.0"
```

---

## 11. Gazebo 모델 설정 주요 사항

### 11.1 무중력 환경
World SDF에서 중력을 0으로 설정:
```xml
<gravity>0 0 0</gravity>
```

### 11.2 우주 배경 (검은 배경 + 조명)
```xml
<scene>
  <ambient>0 0 0 1</ambient>
  <background>0 0 0 1</background>
</scene>

<light type="directional" name="sun">
  <direction>-1 0 -0.3</direction>
</light>
```

### 11.3 관절 제어 플러그인 (model.sdf)

**ApplyJointForce (토크 직접 인가):**
```xml
<plugin filename="gz-sim-apply-joint-force-system"
        name="gz::sim::systems::ApplyJointForce">
  <joint_name>joint1</joint_name>
</plugin>
```

**JointStatePublisher (관절 상태 퍼블리시):**
```xml
<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
  <joint_name>joint1</joint_name>
  <joint_name>joint2</joint_name>
  <!-- ... -->
</plugin>
```

**gz_ros2_control (ROS 2 제어 연동):**
```xml
<plugin filename="gz_ros2_control-system"
        name="gz_ros2_control::GazeboSimROS2ControlPlugin">
  <parameters>/home/ndh/space_ros_ws/install/orbit_sim/share/orbit_sim/config/kari_dual_arm_controllers.yaml</parameters>
  <ros>
    <namespace></namespace>
  </ros>
</plugin>
```
> **주의:** `$(find orbit_sim)` 형식은 Gazebo에서 해석되지 않으므로 절대 경로 사용.

### 11.4 충돌(Collision) 설정

**자기 충돌 비활성화:**
```xml
<self_collide>false</self_collide>
```

**인접 링크 충돌 비활성화 (필수 — 관절 떨림 방지):**
```xml
<disable_collisions>
  <link_pair><link1>satellite_body</link1><link2>arm_L_base</link2></link_pair>
  <link_pair><link1>arm_L_base</link1><link2>arm_L_link1</link2></link_pair>
  <!-- 이하 모든 인접 링크 쌍에 대해 반복 -->
</disable_collisions>
```

### 11.5 관절 파라미터 튜닝
무중력 환경에서 자유로운 움직임을 위해:
```xml
<dynamics>
  <damping>0.0</damping>
  <friction>0.0</friction>
</dynamics>
<limit>
  <lower>-1e16</lower>
  <upper>1e16</upper>
  <velocity>1000</velocity>
</limit>
```

### 11.6 위성 본체 Collision 분리
위성 본체를 본체/태양전지판/전방부로 collision 분리하여 암과의 충돌을 세밀하게 제어:
```xml
<!-- 중앙 본체 -->
<collision name="satellite_body_collision">
  <pose>0 0 0 0 0 0</pose>
  <geometry><box><size>2.0 2.4 2.4</size></box></geometry>
</collision>

<!-- 태양전지판 Left -->
<collision name="satellite_panel_L_collision">
  <pose>0 3.05 0 0 0 0</pose>
  <geometry><box><size>2.0 3.05 0.05</size></box></geometry>
</collision>

<!-- 전방부 (암 장착부) -->
<collision name="satellite_front_collision">
  <pose>1.575 0 0 0 0 0</pose>
  <geometry><box><size>0.8 1.8 1.8</size></box></geometry>
</collision>
```

### 11.7 물리 엔진 설정
```xml
<physics name="default_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```
> `type="dart"`가 무중력 환경에서 안정적. `ode`보다 6DOF 제어에 적합.

---

## 12. 트러블슈팅

### 12.1 빌드 관련

| 증상 | 해결 |
|------|------|
| `qt_gui` 패키지 충돌 | src에 `qt_gui_core` 소스가 있으면 발생. orbit_sim만 남기면 해결 |
| `warehouse_ros_mongo` 의존성 | src에 `demos`가 있을 때만 발생. orbit_sim만 남기면 해결 |

### 12.2 GPU 렌더링 관련

| 증상 | 해결 |
|------|------|
| `OpenGL renderer: llvmpipe` (소프트웨어 렌더링) | `.bashrc`에 `GALLIUM_DRIVER=d3d12` 확인 |
| `nvidia-smi` 동작하나 GL 안 잡힘 | `sudo apt install --reinstall libegl-mesa0 libgl1-mesa-dri` |
| `.bashrc`에 중복 환경변수 | `sed -i '/변수명/d' ~/.bashrc` 로 정리 후 재설정 |
| Gazebo GUI 검은 화면 | `nvidia-smi`로 드라이버 확인, WSL 커널 업데이트 |

### 12.3 모델/시뮬레이션 관련

| 증상 | 해결 |
|------|------|
| 모델 로딩 안 됨 | `echo $GZ_SIM_RESOURCE_PATH` 확인, models 디렉토리 포함 여부 |
| mesh 파일 못 찾음 | model.sdf에서 `package://` 대신 상대경로 `meshes/xxx.stl` 사용 |
| 관절 떨림/폭발 | `<self_collide>false` + `<disable_collisions>` 인접 링크 설정 |
| gz_ros2_control 플러그인 안 로드 | `GZ_SIM_SYSTEM_PLUGIN_PATH`에 `/opt/ros/jazzy/lib` 추가 |
| controller yaml 경로 오류 | `$(find ...)` 대신 절대 경로 사용 |

### 12.4 ROS 통신 관련

| 증상 | 해결 |
|------|------|
| 토픽이 안 보임 (다른 터미널) | `source install/setup.bash` 확인 |
| DDS 통신 문제 | `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| ROS_DOMAIN_ID 충돌 | `export ROS_DOMAIN_ID=0` (기본값) |

---

## 13. 향후 확장 방향

현재 시뮬레이션은 Gazebo 정적 무중력 환경에서의 듀얼암 도킹 시뮬레이션이며, GAiMSat-1 미션을 위해 다음 확장을 고려:

### 13.1 궤도역학 연동
- Gazebo 커스텀 System 플러그인(C++)으로 CW/HCW 방정식 기반 외력 주입
- 또는 외부 궤도 전파기 ROS 2 노드 + Gazebo pose 동기화
- MATLAB에서 구현한 J2 섭동 포함 궤도 전파기를 Python/C++ 노드로 포팅

### 13.2 센서 시뮬레이션
- 근접 항법용 카메라 센서 (실제 intrinsic/distortion 파라미터)
- IMU 센서 (bias, noise 모델 포함)
- Star Tracker 시뮬레이션 (별 배경 skybox + 별 카탈로그)

### 13.3 Target 위성 모델
- 비협조 Target 위성 (텀블링 초기 각속도 부여)
- 도킹 포트/그래플 포인트 형상
- 비대칭 관성 텐서 설정

### 13.4 조명 환경
- 궤도 위치에 따른 동적 태양광 방향 변경 플러그인
- Eclipse 시뮬레이션 (태양광 on/off)
- 지구 반사광(Earthshine) 근사

### 13.5 추천 아키텍처
```
[Orbit Propagator Node]     [Sensor Sim Node]
   (J2, CW equations)          (Camera, IMU, ST)
         │                           │
         ▼                           ▼
   /chaser_state_true          /camera/image
   /target_state_true          /imu/data
         │                           │
         ▼                           ▼
[Gazebo Harmonic] ◄── gz_bridge ──► [ROS 2 Topics]
   - Visual rendering                    │
   - Contact physics (docking)           ▼
   - Joint dynamics (arm)         [Nav/Control Nodes]
                                    - ESKF
                                    - MPPI Controller
                                    - Path Planner
```

---

## 부록: 참고 자료

- [ROS 2 Jazzy 공식 문서](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic 공식 문서](https://gazebosim.org/docs/harmonic)
- [Space ROS 공식 리포지토리](https://github.com/space-ros)
- [ros_gz 브리지 문서](https://github.com/gazebosim/ros_gz)
- [WSL2 GPU 가속 설정](https://learn.microsoft.com/windows/wsl/tutorials/gpu-compute)
- [spaceros_gz_demos (NASA Space ROS)](https://github.com/david-dorf/spaceros_gz_demos)
