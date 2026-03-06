# Stereo Vision Gazebo 테스트 환경 구축 플랜

> **목적**: GaimSat-1 고정밀 3D 모델을 Gazebo Harmonic에 배치하고, 기존 Deputy 위성(nasa_satellite5)의 3쌍 스테레오 카메라로 촬영하여 스테레오 비전 알고리즘을 검증하는 시뮬레이션 환경 구축
>
> **위치**: `~/space_ros_ws/src/orbit_sim/`
> **작성일**: 2026-03-02

---

## 1. 현황 분석

### 1.1 기존 자산

| 자산 | 위치 | 상태 |
|------|------|------|
| Deputy 위성 모델 (nasa_satellite5) | `models/nasa_satellite5/` | ✅ 스테레오 3쌍 장착 완료 |
| Chief 위성 모델 (intel_sat_dummy) | `models/intel_sat_dummy/` | ✅ 단순 더미 |
| GAIM 3U 타겟 (gaim_target_3u) | `models/gaim_target_3u/` | ✅ 저해상도 DAE |
| LVLH GCO 월드 | `worlds/orbit_LVLH_GCO.world` | ✅ 제로중력, DART |
| 카메라 테스트 런처 | `launch/gco_cam_test.launch.py` | ✅ 스테레오 브리지 완비 |
| GaimSat OBJ (고정밀) | `Space_SLAM/model/obj_read_test/GAIM_OBJ/` | ✅ 174MB, 2.4M vertices |

### 1.2 기존 스테레오 카메라 사양 (nasa_satellite5)

| 쌍 | FOV | 해상도 | 포맷 (L/R) | 베이스라인 | 용도 |
|----|-----|--------|-----------|----------|------|
| **NFOV** | 5.7° (0.1 rad) | 1024×1024 | RGB / L8 | 0.2m (Y) | 장거리 (>5km) |
| **WFOV** | 19.5° (0.34 rad) | 1024×1024 | L8 / RGB | 0.2m (Y) | 중거리 (1~5km) |
| **Docking** | 68° (1.19 rad) | 1024×1024 | L8 / RGB | 0.2m (Y) | 근거리 (<1km) |

### 1.3 기존 토픽 구조

```
/mev/vss_nfov/{left,right}/image_raw     → sensor_msgs/Image
/mev/vss_wfov/{left,right}/image_raw     → sensor_msgs/Image
/mev/vss_docking/{left,right}/image_raw  → sensor_msgs/Image
/nasa_satellite5/imu                      → sensor_msgs/Imu
/model/nasa_satellite5/odometry           → nav_msgs/Odometry
```

---

## 2. 해결해야 할 문제

### 2.1 GaimSat 메시 변환 (OBJ → Gazebo 호환)

| 문제 | 설명 | 해결책 |
|------|------|--------|
| **포맷** | OBJ/MTL → Gazebo는 DAE(Collada) 또는 OBJ 지원 | Blender에서 DAE 내보내기 권장 |
| **크기** | 174MB, 2.4M vertices → 실시간 렌더링 불가능 | Decimate modifier (10~20% 유지) |
| **방향** | Roll -90° 필요 | Blender에서 변환 후 내보내기 |
| **스케일** | 0.59×0.76m (실제 6U 크기) | 그대로 유지 (미터 단위) |
| **재질** | 71개 MTL 재질 → Collada material | Blender 자동 변환 |

**메시 LOD 전략**:
```
GaimSat_high.dae   — ~500K faces (visual only, optional)
GaimSat_mid.dae    — ~100K faces (기본 visual)
GaimSat_low.dae    — ~20K faces (collision용)
```

### 2.2 카메라 파라미터 갱신 (GAIMSat-1 실제 사양)

현재 nasa_satellite5의 카메라는 임시 사양. **CLAUDE.md의 실제 GAIMSat-1 카메라 파라미터**로 교체 필요:

| 항목 | 현재 (nasa_satellite5) | 목표 (GAIMSat-1 사양) |
|------|----------------------|---------------------|
| NFOV FOV | 0.1 rad (5.7°) | 0.523 rad (30°) |
| WFOV FOV | 0.34 rad (19.5°) | 1.082 rad (62°) |
| Baseline | 0.2m | 0.07m (70mm) |
| 센서 | 1024×1024 | 3280×2464 (IMX219) or 1024×1024 (축소) |
| 프레임레이트 | 4 Hz | 5 Hz (시뮬레이션 주기) |

> **판단 포인트**: 실제 사양으로 교체할지, 기존 유지하고 별도 모델로 만들지 결정 필요

---

## 3. 구현 플랜

### Phase A: GaimSat Gazebo 모델 생성

#### A-1. Blender 메시 변환 (수동 작업)

```
1. Blender에서 GaimSat.obj 로드
2. Roll -90° 적용 (X축 기준)
3. Decimate modifier 적용:
   - ratio 0.05 → ~100K faces (visual)
   - ratio 0.01 → ~20K faces (collision)
4. DAE (Collada) 내보내기
   - ✅ Apply Modifiers
   - ✅ Include Material Textures
   - ✅ Triangulate
5. 출력 → models/gaimsat_target/meshes/
```

#### A-2. model.config 생성

```xml
<?xml version="1.0"?>
<model>
  <name>gaimsat_target</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>NDH</name>
  </author>
  <description>GAIMSat-1 high-fidelity target satellite (6U CubeSat)</description>
</model>
```

#### A-3. model.sdf 생성

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="gaimsat_target">
    <pose>0 0 0 0 0 0</pose>
    <static>false</static>
    <link name="gaimsat_link">
      <inertial>
        <mass>10.5</mass>  <!-- 6U CubeSat 10.5 kg -->
        <inertia>
          <!-- 6U: 0.34×0.23×0.10 m → 균일 직육면체 가정 -->
          <ixx>0.055</ixx>
          <iyy>0.104</iyy>
          <izz>0.127</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://gaimsat_target/meshes/gaimsat_mid.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://gaimsat_target/meshes/gaimsat_low.dae</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

#### A-4. 파일 구조

```
models/gaimsat_target/
├── model.config
├── model.sdf
└── meshes/
    ├── gaimsat_mid.dae     ← visual (~100K faces)
    └── gaimsat_low.dae     ← collision (~20K faces)
```

---

### Phase B: 스테레오 테스트 월드 생성

#### B-1. `worlds/stereo_test.world`

기존 `orbit_LVLH_GCO.world` 기반으로:
- Chief → `gaimsat_target` (원점, static)
- Deputy → `nasa_satellite5` (초기 위치 조절 가능)
- 조명: 태양광 (우주 조건 모사)
- 배경: 검정 (우주)
- 물리: DART, 제로중력

**테스트 거리 시나리오**:

| 시나리오 | Deputy 위치 | 사용 카메라 | 목적 |
|---------|-----------|-----------|------|
| Far (5km) | (0, 500, 0) | NFOV | 장거리 감지 |
| Mid (1km) | (0, 100, 0) | WFOV | 중거리 추적 |
| Close (150m) | (0, 15, 0) | WFOV + Docking | 근접 관측 |
| Dock (20m) | (0, 2, 0) | Docking | 도킹 접근 |

> 월드 내 스케일: Gazebo 단위 1 = 실제 ~10m (기존 관례)

#### B-2. 월드 핵심 구조

```xml
<?xml version="1.0"?>
<sdf version="1.10">
  <world name="stereo_test_world">
    <!-- 물리: DART, 제로중력 -->
    <physics name="orbital_physics" type="dart" default="true">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <gravity>0 0 0</gravity>
    <scene>
      <background>0 0 0 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <!-- 태양광 -->
    <light type="directional" name="sun">
      <pose>-8000 -2000 4000 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>0.8 0.2 -0.4</direction>
    </light>

    <!-- GaimSat 타겟 (Chief, 원점 고정) -->
    <include>
      <uri>model://gaimsat_target</uri>
      <name>chief_gaimsat</name>
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
    </include>

    <!-- Deputy (스테레오 카메라 탑재) -->
    <include>
      <uri>model://nasa_satellite5</uri>
      <name>deputy_satellite</name>
      <pose>0 15 0 0 0 0</pose>  <!-- 150m 거리 -->
    </include>

    <!-- GUI: 3D + 스테레오 이미지 디스플레이 -->
    <gui>
      <plugin filename="GzScene3D" name="3D View">
        <engine>ogre2</engine>
        <background_color>0 0 0 1</background_color>
      </plugin>
      <plugin filename="ImageDisplay" name="Stereo Left">
        <topic>/mev/vss_wfov/left/image_raw</topic>
      </plugin>
      <plugin filename="ImageDisplay" name="Stereo Right">
        <topic>/mev/vss_wfov/right/image_raw</topic>
      </plugin>
    </gui>

    <!-- 시스템 플러그인 -->
    <plugin filename="libgz-sim-sensors-system.so" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libgz-sim-physics-system.so" name="gz::sim::systems::Physics"/>
    <plugin filename="libgz-sim-user-commands-system.so" name="gz::sim::systems::UserCommands"/>
    <plugin filename="libgz-sim-scene-broadcaster-system.so" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="libgz-sim-imu-system.so" name="gz::sim::systems::Imu"/>
    <plugin filename="libgz-sim-apply-link-wrench-system.so" name="gz::sim::systems::ApplyLinkWrench"/>
  </world>
</sdf>
```

---

### Phase C: 런치 파일 생성

#### C-1. `launch/stereo_test.launch.py`

기존 `gco_cam_test.launch.py` 패턴 기반:

```python
def generate_launch_description():
    orbit_sim_path = get_package_share_directory('orbit_sim')
    world_file = os.path.join(orbit_sim_path, 'worlds', 'stereo_test.world')
    model_path = os.path.join(orbit_sim_path, 'models')

    env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    }

    # 1. Gazebo 시뮬레이션
    start_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        additional_env=env
    )

    # 2. 스테레오 카메라 브리지 (6개 토픽)
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/mev/vss_nfov/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_nfov/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_wfov/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_wfov/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_docking/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mev/vss_docking/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        ]
    )

    # 3. IMU + Odometry 브리지
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/nasa_satellite5/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ]
    )

    # 4. SetEntityPose 서비스 브리지
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/stereo_test_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ]
    )

    # 5. 포즈 제어 GUI (기존 pose_control_camtest 재사용)
    pose_gui = Node(
        package='orbit_sim',
        executable='pose_control_camtest',
        name='stereo_pose_control',
    )

    # 6. (선택) 스테레오 처리 노드
    # stereo_processor = Node(
    #     package='orbit_sim',
    #     executable='stereo_processor',
    # )

    return LaunchDescription([
        start_world,
        camera_bridge,
        imu_bridge,
        pose_bridge,
        pose_gui,
    ])
```

#### C-2. 런치 인자 (옵션)

```python
# 거리 시나리오 선택
DeclareLaunchArgument('distance', default_value='150',
    description='Initial deputy distance in meters (20/150/1000/5000)')
```

---

### Phase D: 스테레오 처리 노드 (선택, 후속)

#### D-1. `orbit_sim/stereo_processor.py`

```python
class StereoProcessor(Node):
    def __init__(self):
        super().__init__('stereo_processor')

        # 파라미터
        self.declare_parameter('stereo_pair', 'wfov')  # nfov/wfov/docking
        self.declare_parameter('algorithm', 'sgm')      # bm/sgm

        # 좌/우 이미지 동기 구독 (message_filters)
        self.left_sub = Subscriber(self, Image, '/mev/vss_wfov/left/image_raw')
        self.right_sub = Subscriber(self, Image, '/mev/vss_wfov/right/image_raw')
        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=5, slop=0.1)
        self.sync.registerCallback(self.stereo_callback)

        # 디스패리티 맵 퍼블리셔
        self.disp_pub = self.create_publisher(Image, '/stereo/disparity', 10)
        # 포인트 클라우드 퍼블리셔
        self.pc_pub = self.create_publisher(PointCloud2, '/stereo/pointcloud', 10)

    def stereo_callback(self, left_msg, right_msg):
        # cv_bridge → OpenCV
        left = self.bridge.imgmsg_to_cv2(left_msg)
        right = self.bridge.imgmsg_to_cv2(right_msg)

        # 스테레오 매칭 (OpenCV StereoSGBM)
        disparity = self.compute_disparity(left, right)

        # 3D 복원
        points_3d = cv2.reprojectImageTo3D(disparity, self.Q)

        # 퍼블리시
        self.disp_pub.publish(self.bridge.cv2_to_imgmsg(disparity))
```

#### D-2. setup.py 업데이트

```python
'stereo_processor = orbit_sim.stereo_processor:main',
```

---

## 4. 작업 순서 (체크리스트)

### Step 1: 메시 변환 (Blender 수동)
- [ ] GaimSat.obj를 Blender에서 열기
- [ ] Roll -90° 적용 (X축)
- [ ] Decimate: visual ~100K, collision ~20K
- [ ] DAE 내보내기 → `gaimsat_mid.dae`, `gaimsat_low.dae`
- [ ] 파일 크기/품질 확인

### Step 2: Gazebo 모델 생성
- [ ] `models/gaimsat_target/` 디렉토리 생성
- [ ] `model.config` 작성
- [ ] `model.sdf` 작성 (관성, visual, collision)
- [ ] meshes/ 에 DAE 파일 배치
- [ ] `gz model --list` 또는 단독 테스트로 로딩 확인

### Step 3: 월드 파일 생성
- [ ] `worlds/stereo_test.world` 작성
- [ ] GaimSat 타겟 + Deputy 배치
- [ ] 조명/배경/물리 설정
- [ ] GUI ImageDisplay 플러그인 (좌/우 이미지)
- [ ] `gz sim stereo_test.world` 단독 실행 확인

### Step 4: 런치 파일 생성
- [ ] `launch/stereo_test.launch.py` 작성
- [ ] 카메라 6개 토픽 브리지
- [ ] IMU + SetEntityPose 브리지
- [ ] pose_control_camtest 노드 연결
- [ ] 월드명 일치 확인 (`stereo_test_world`)

### Step 5: 빌드 & 테스트
- [ ] `colcon build --symlink-install --packages-select orbit_sim`
- [ ] `source install/setup.bash`
- [ ] `ros2 launch orbit_sim stereo_test.launch.py`
- [ ] 좌/우 이미지 토픽 echo 확인
- [ ] `ros2 topic hz /mev/vss_wfov/left/image_raw` (4 Hz 확인)
- [ ] rqt_image_view로 스테레오 이미지 육안 확인

### Step 6: (후속) 스테레오 처리 노드
- [ ] `stereo_processor.py` 작성
- [ ] setup.py entry_points 추가
- [ ] rebuild + 실행 테스트
- [ ] 디스패리티 맵 시각화 확인

---

## 5. 주의사항

### 5.1 메시 크기 제약
- **Gazebo Harmonic + ogre2**: 500K faces 이상이면 프레임 드롭 심각
- GaimSat 원본 2.2M faces → 반드시 Decimate 필요
- WSL2 GPU 패스스루(D3D12): 렌더링 성능 추가 제한

### 5.2 좌표계 주의
- **Gazebo**: X-forward, Y-left, Z-up (ROS 표준)
- **GaimSat OBJ**: Blender 좌표계 (Z-up, Y-forward) → Roll -90° 필요
- **카메라 센서**: Gazebo 카메라는 +X 방향을 바라봄
- **LVLH**: 기존 월드와 동일한 좌표 관례 유지

### 5.3 월드명 일치
- 월드 SDF의 `<world name="stereo_test_world">` 와
- 런치 파일의 `set_pose` 서비스 경로 `/world/stereo_test_world/set_pose` 가 반드시 일치해야 함

### 5.4 카메라 파라미터 변경 시
- nasa_satellite5/model.sdf 직접 수정 → 기존 런처 영향
- **권장**: 별도 모델(`mev_deputy_v2`) 생성하여 GAIMSat-1 실제 사양 적용
- 또는 기존 모델 유지하고 후속 단계에서 교체

### 5.5 빌드 순서
```bash
cd ~/space_ros_ws
colcon build --symlink-install --packages-select orbit_sim
source install/setup.bash
```
- `models/`, `worlds/`, `launch/` 변경 시 **반드시 rebuild**
- Python 노드만 변경 시 `--symlink-install` 덕분에 rebuild 불필요

---

## 6. 예상 결과

### 성공 기준
1. Gazebo에서 GaimSat 고정밀 모델이 정상 렌더링됨
2. Deputy 위성의 스테레오 카메라 6개 토픽에서 이미지 수신 가능
3. 좌/우 이미지에서 GaimSat 대상체가 보임
4. pose_control_camtest GUI로 거리/각도 실시간 변경 가능
5. (후속) 디스패리티 맵에서 3D 깊이 복원 확인

### 검증할 스테레오 시나리오 (study/stereo_vision V1~V6 연계)

| Gazebo 테스트 | 연계 검증 | 핵심 확인 |
|--------------|----------|----------|
| 150m WFOV 스테레오 | V1 깊이 정밀도 | σ_Z 측정 |
| 150m NFOV+WFOV 동시 | V5 Foveated 교차검증 | WN vs WW 비교 |
| 20m Docking 스테레오 | V6 FOV 트레이드오프 | 근거리 정밀도 |
| 다양한 조명 각도 | V4 저조도 강건성 | SNR별 매칭 품질 |

---

*이 플랜은 Step 1 (Blender 메시 변환)을 먼저 수행한 후 순차 진행.*
