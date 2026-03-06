# Stereo Vision Test Environment — 작동 안내서

## 개요

GAIMSat-1(6U Deputy)의 스테레오 카메라 3쌍으로 3U CubeSat(Chief/Target)을 관측하는 Gazebo 시뮬레이션 환경.
GUI에서 포즈를 조작하고, 촬영 버튼으로 스테레오 이미지 쌍을 캡처하여 파일로 저장한다.

### 배치 구조

```
Chief/Target (원점, 고정)          Deputy/Observer (Y=15, 이동 가능)
┌─────────────┐                   ┌─────────────────┐
│ gaim_target_3u │  ◄── 관측 ──  │  gaimsat_target  │
│   3U CubeSat   │               │  6U GaimSat      │
│   (static)     │               │  [NFOV][WFOV]    │
└─────────────┘                   │  [Docking]       │
    (0, 0, 0)                     └─────────────────┘
                                      (0, 15, 0)
```

---

## 실행

```bash
cd ~/space_ros_ws
colcon build --symlink-install --packages-select orbit_sim
source install/setup.bash
ros2 launch orbit_sim stereo_test.launch.py
```

실행 시 자동으로 뜨는 것:
- **Gazebo 창** — 3D 뷰 + WFOV 좌/우 이미지 디스플레이
- **Tkinter GUI** — 포즈 제어 + 촬영 버튼

---

## GUI 사용법

### 거리 프리셋

| 버튼 | Deputy 위치 (Y) | 실거리 (×10) | 적합 카메라 |
|------|----------------|-------------|------------|
| 5 km | 500 | 5,000 m | NFOV |
| 1 km | 100 | 1,000 m | NFOV / WFOV |
| 150 m | 15 | 150 m | WFOV |
| 20 m | 2 | 20 m | WFOV / Docking |
| 2 m | 0.2 | 2 m | Docking |

> Gazebo 단위 1 = 실제 약 10m (orbit_sim 관례)

### Deputy Pose

| 필드 | 단위 | 설명 |
|------|------|------|
| X, Y, Z | Gazebo unit | Deputy 위치 |
| Roll, Pitch, Yaw | radian | Deputy 자세 |

입력 후 **Apply Deputy** 클릭.

### Target Rotation

Chief(3U CubeSat)의 자세를 변경하여 다른 면을 카메라에 노출.

| 필드 | 단위 | 설명 |
|------|------|------|
| Roll, Pitch, Yaw | radian | 타겟 회전 |

입력 후 **Apply Target** 클릭.

> 참고: 90° = 1.5708 rad, 180° = 3.1416 rad

### 촬영 (CAPTURE)

1. 원하는 거리/각도로 Deputy와 Target 포즈 설정
2. 우측 **Image Buffers** 에서 6개 모두 `OK` 상태 확인
3. **CAPTURE** 버튼 클릭
4. `~/stereo_captures/YYYYMMDD_HHMMSS/` 에 저장됨

#### 저장 파일 구조

```
~/stereo_captures/20260302_143022/
├── nfov_left.png       # NFOV 좌 (RGB, 1024×1024)
├── nfov_right.png      # NFOV 우 (Grayscale, 1024×1024)
├── wfov_left.png       # WFOV 좌 (Grayscale, 1024×1024)
├── wfov_right.png      # WFOV 우 (RGB, 1024×1024)
├── docking_left.png    # Docking 좌 (Grayscale, 1024×1024)
├── docking_right.png   # Docking 우 (RGB, 1024×1024)
└── metadata.json       # 포즈, 거리, 카메라 파라미터
```

#### metadata.json 예시

```json
{
  "timestamp": "20260302_143022",
  "deputy_pose": { "x": 0.0, "y": 15.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0 },
  "target_rotation": { "roll": 0.0, "pitch": 0.0, "yaw": 0.0 },
  "distance_gz_unit": 15.0,
  "distance_m": 150.0,
  "cameras": {
    "nfov":    { "fov_rad": 0.523, "f_px": 3843.47, "baseline_m": 0.07, "expected_disparity_px": 1.794 },
    "wfov":    { "fov_rad": 1.082, "f_px": 1755.2,  "baseline_m": 0.07, "expected_disparity_px": 0.819 },
    "docking": { "fov_rad": 1.19,  "f_px": 803.07,  "baseline_m": 0.07, "expected_disparity_px": 0.375 }
  }
}
```

---

## 카메라 사양 (GAIMSat-1)

| 쌍 | FOV | f_px (1024 기준) | 베이스라인 | 포맷 L/R | clip | rate |
|----|-----|-----------------|-----------|---------|------|------|
| **NFOV** | 0.523 rad (30°) | 3,843 | 70mm | RGB / L8 | 1~50km | 5Hz |
| **WFOV** | 1.082 rad (62°) | 1,755 | 70mm | L8 / RGB | 0.1~50km | 5Hz |
| **Docking** | 1.19 rad (68°) | 803 | 70mm | L8 / RGB | 0.1~50km | 5Hz |

카메라 마운트 위치 (gaimsat_link 기준):
- NFOV: `(0, -0.40, +0.05)` — 상단
- WFOV: `(0, -0.40, 0)` — 중앙
- Docking: `(0, -0.40, -0.05)` — 하단

모두 -Y 면(메시 바깥)에서 -Y 방향(타겟 방향)을 향함.

디스패리티 공식: **d = f_px × B / Z**

---

## ROS 2 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mev/vss_nfov/left/image_raw` | sensor_msgs/Image | NFOV 좌 (RGB) |
| `/mev/vss_nfov/right/image_raw` | sensor_msgs/Image | NFOV 우 (L8) |
| `/mev/vss_wfov/left/image_raw` | sensor_msgs/Image | WFOV 좌 (L8) |
| `/mev/vss_wfov/right/image_raw` | sensor_msgs/Image | WFOV 우 (RGB) |
| `/mev/vss_docking/left/image_raw` | sensor_msgs/Image | Docking 좌 (L8) |
| `/mev/vss_docking/right/image_raw` | sensor_msgs/Image | Docking 우 (RGB) |
| `/model/gaimsat_target/odometry` | nav_msgs/Odometry | Deputy 위치/자세 |

### 서비스

| 서비스 | 타입 | 용도 |
|--------|------|------|
| `/world/stereo_test_world/set_pose` | SetEntityPose | 엔티티 포즈 제어 |

### 엔티티 이름

| 이름 | 모델 | 역할 |
|------|------|------|
| `chief_target` | gaim_target_3u | 타겟 (3U, 원점) |
| `deputy_gaimsat` | gaimsat_target | 관측자 (6U, 카메라) |

---

## 토픽 확인 명령

```bash
# 토픽 목록
ros2 topic list | grep mev

# 이미지 주파수 확인
ros2 topic hz /mev/vss_wfov/left/image_raw

# 이미지 시각화
ros2 run rqt_image_view rqt_image_view

# Deputy 위치 확인
ros2 topic echo /model/gaimsat_target/odometry --once
```

---

## 파일 구성

```
orbit_sim/
├── models/gaimsat_target/
│   ├── model.config
│   ├── model.sdf              ← 스테레오 3쌍 + Odom + VelCtrl
│   └── meshes/
│       ├── gaimsat_mid.dae    ← visual (0.59×0.76×0.37m)
│       └── gaimsat_low.dae
├── models/gaim_target_3u/
│   ├── model.config
│   ├── model.sdf
│   └── meshes/target_3u.dae   ← 3U CubeSat 타겟
├── worlds/stereo_test.world
├── launch/stereo_test.launch.py
├── orbit_sim/stereo_test_gui.py
└── docs/stereo_test_guide.md   ← 이 파일
```

---

## 종료

GUI **Quit** 버튼 또는 터미널에서 `Ctrl+C`.

프로세스가 남아있을 경우:
```bash
pkill -9 -f "gz sim"
pkill -9 -f "stereo_test"
pkill -9 -f "parameter_bridge"
```
