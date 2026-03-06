# orbit_sim - WSL2 위성 근접운용 시뮬레이션

> ROS 2 Jazzy + Gazebo Harmonic 기반 위성 근접운용 시뮬레이션 패키지.
> 듀얼암 도킹, LVLH 상대운동, 편대비행, 비전 기반 자세추정을 포괄합니다.

---

## 필수: 작업 시작 전 확인사항

**코드 수정 전 반드시 확인:**

1. **파일 Read 먼저**: 반드시 Read → 전체 흐름 파악 → 수정 (추측 코딩 금지)
2. **빌드 확인**: 수정 후 `colcon build --packages-select orbit_sim` 성공 여부 확인
3. **좌표계 혼동 금지**: LVLH / ECI / Body 프레임 구분, 쿼터니언 scalar-first 엄수

- [ ] 위 3가지를 현재 작업에 대입해 확인했는가?

**이 단계를 건너뛰고 코드를 수정하는 것은 금지입니다.**

---

## 개발 환경

| 항목 | 값 |
|------|------|
| OS | Windows 10 Pro (WSL2 Ubuntu 24.04) |
| CPU | AMD Ryzen 5 5600 (6C/12T) |
| RAM | 32 GB (WSL2: 8GB 제한) |
| GPU | NVIDIA RTX 3060 (12 GB VRAM) — D3D12 렌더링 |
| IDE | VSCode + Claude Code Extension (WSL Remote) |
| Shell | bash (WSL2) |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (DART 물리엔진) |
| Python | 3.x (시스템) |
| 빌드 | colcon build --symlink-install |

---

## 핵심 실행 명령어

| 시뮬레이션 | 명령어 |
|----------|--------|
| KARI 단일암 | `ros2 launch orbit_sim kari_arm.launch.py` |
| KARI 듀얼암 | `ros2 launch orbit_sim kari_dual_arm.launch.py` |
| 듀얼암 도킹 | `ros2 launch orbit_sim docking_sim.launch.py` |
| LVLH 상대운동 | `ros2 launch orbit_sim lvlh_sim.launch.py` |
| 편대비행 | `ros2 launch orbit_sim fm_fly.launch.py` |
| GCO 테스트 | `ros2 launch orbit_sim gco_test.launch.py` |
| GCO + LiDAR + DVS | `ros2 launch orbit_sim gco_lidar_mapping.launch.py` |
| PnP 테스트 | `ros2 launch orbit_sim pnp_test.launch.py` |

---

## 코드 규칙

### 스타일

- 기존 코드 스타일 유지 (변수명/함수명 임의 수정 금지)
- 주석: 한국어 + 영어 혼용 (UTF-8)
- ROS 2 노드: `rclpy` 기반, `Node` 상속
- 파라미터: `self.declare_parameter()` 사용
- 노드 1파일 = 1노드 (entry_points in setup.py)
- 토픽/서비스 이름: `snake_case`
- Launch 파일: `*.launch.py` (Python launch)
- Gazebo 모델: `model.sdf` + `model.config` 패턴

### 수정 범위 제한 (엄수)

- **수정 전 반드시 사용자 허락 필수**
- **요청된 코드만 수정**. 인접 코드의 리팩터링, 주석 추가, 포맷 정리 금지
- **Dead code 발견 시**: 삭제하지 않고 사용자에게 위치와 내용을 보고만 한다
- **알고리즘 적용 전**: "이 알고리즘은 [전제조건]을 가정합니다. 현재 시뮬레이션은 [상태]입니다" 형태로 전제조건-현실 대조를 먼저 명시한다
- 좌표계 변환 체인 중간 단계 생략 금지
- 물리 파라미터(관성, 감쇠) 변경 시 시뮬레이션 안정성 확인
- setup.py의 entry_points 변경 시 rebuild 필수
- World/Model SDF 변경 시 `GZ_SIM_RESOURCE_PATH` 경로 확인

---

## 프로세스 정리

시뮬레이션 종료 후 **반드시** 아래 스크립트로 전체 프로세스를 정리한다.
`pkill`은 패턴 매칭 실패로 프로세스가 남는 경우가 있으므로, 항상 이 스크립트를 사용할 것.

```bash
bash ~/kill_sim.sh
```

> 스크립트 위치: `~/kill_sim.sh`
> 대상: gz sim, parameter_bridge, multi_satellite_controller, pointcloud_mapper, rviz2, rqt_image_view 등
> 방식: `pgrep` → PID 수집 → `kill -9` (2회 반복) → 잔존 프로세스 경고

---

## 빌드 워크플로우

```
Step 1: 코드 수정
  → orbit_sim/orbit_sim/*.py 또는 launch/*.py 수정

Step 2: 빌드
  → cd ~/space_ros_ws
  → colcon build --symlink-install --packages-select orbit_sim
  → source install/setup.bash

Step 3: 실행
  → ros2 launch orbit_sim <launch_file>.launch.py

Step 4: 디버그
  → ros2 topic list / ros2 topic echo
  → ros2 node list / ros2 service list
```

> `--symlink-install`로 빌드하면 Python 파일은 심볼릭 링크 → 수정 즉시 반영.
> 단, `setup.py`, `package.xml`, `launch/` 변경 시 rebuild 필수.

---

## 연관 프로젝트

| 프로젝트 | 위치 | 관계 |
|----------|------|------|
| Space_SLAM | `D:\space_vision\Space_SLAM` (Windows) | MATLAB 기반 알고리즘 연구 (필터, SLAM, 3DGS) |
| orbit_sim | `/home/ndh/space_ros_ws/src/orbit_sim` (WSL2) | ROS 2 기반 시뮬레이션 구현 |
| gz_dvs_plugin | `/home/ndh/space_ros_ws/src/gz_dvs_plugin` (WSL2) | DVS 이벤트 카메라 gz-sim 플러그인 (별도 ament_cmake 패키지) |

Space_SLAM에서 검증된 알고리즘 → orbit_sim에서 ROS 2 노드로 구현하는 흐름.

> 상세 좌표계/쿼터니언/단위 컨벤션은 `orbit_sim/CLAUDE.md` 참조.
> Gazebo 모델/World 설정 가이드는 `models/CLAUDE.md` 참조.

---

## 개발 진행 상황

| Phase | 항목 | 상태 |
|-------|------|------|
| 1 | WSL2 + ROS 2 Jazzy + Gazebo Harmonic 환경 구축 | 완료 |
| 2 | orbit_sim 패키지 구조 설계 | 완료 |
| 3 | Gazebo 모델 (위성, 로봇암, 행성) | 완료 |
| 4 | KARI 암 임피던스 제어 (단일/듀얼) | 완료 |
| 5 | LVLH 상대운동 시뮬레이션 | 완료 |
| 6 | 편대비행 다중 위성 제어 | 완료 |
| 7 | ArUco 비전 자세추정 | 완료 |
| 8 | GCO 제어 + 카메라 테스트 | 완료 |
| 9 | HTTP-ROS 브리지 (웹 연동) | 완료 |
| 10 | LiDAR 3D 매핑 (DragonEye/TriDAR급) | 완료 |
| 11 | DVS 이벤트 카메라 플러그인 (`gz_dvs_plugin`) | 완료 |
| 12 | Space_SLAM 필터 알고리즘 ROS 이식 | 예정 |
| 13 | 3DGS 밀집 표현 ROS 통합 | 예정 |

---

## 작업 기록 규칙

- 실수 발생 시 사용자에게 즉시 보고하고 원인 분석
- 주요 변경사항은 git commit으로 기록

---

*최종 업데이트: 2026-03-06*
