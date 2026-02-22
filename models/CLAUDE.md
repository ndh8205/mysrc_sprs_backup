# Gazebo 모델 / World 설정 가이드

> 이 파일은 `orbit_sim/models/` 및 `orbit_sim/worlds/` 디렉토리의 Gazebo 모델 설정 시 참조할 가이드입니다.

---

## 무중력 환경

World SDF에서 중력을 0으로 설정:
```xml
<gravity>0 0 0</gravity>
```

## 우주 배경 (검은 배경 + 조명)

```xml
<scene>
  <ambient>0 0 0 1</ambient>
  <background>0 0 0 1</background>
</scene>

<light type="directional" name="sun">
  <direction>-1 0 -0.3</direction>
</light>
```

---

## 물리엔진 (DART)

```xml
<physics name="default_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

> `type="dart"`가 무중력 환경에서 안정적. `ode`보다 6DOF 제어에 적합.

---

## gz_ros2_control 플러그인

```xml
<plugin filename="gz_ros2_control-system"
        name="gz_ros2_control::GazeboSimROS2ControlPlugin">
  <parameters>/home/ndh/space_ros_ws/install/orbit_sim/share/orbit_sim/config/kari_dual_arm_controllers.yaml</parameters>
  <ros>
    <namespace></namespace>
  </ros>
</plugin>
```

> **주의:** `$(find orbit_sim)` 형식은 Gazebo에서 해석되지 않으므로 **절대 경로** 사용.
> `GZ_SIM_SYSTEM_PLUGIN_PATH`에 `/opt/ros/jazzy/lib` 필수.

---

## 관절 제어 플러그인

### ApplyJointForce (토크 직접 인가)

```xml
<plugin filename="gz-sim-apply-joint-force-system"
        name="gz::sim::systems::ApplyJointForce">
  <joint_name>joint1</joint_name>
</plugin>
```

### JointStatePublisher (관절 상태 퍼블리시)

```xml
<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
  <joint_name>joint1</joint_name>
  <joint_name>joint2</joint_name>
  <!-- ... -->
</plugin>
```

---

## 충돌(Collision) 설정

### 자기 충돌 비활성화

```xml
<self_collide>false</self_collide>
```

### 인접 링크 충돌 비활성화 (필수 — 관절 떨림 방지)

```xml
<disable_collisions>
  <link_pair><link1>satellite_body</link1><link2>arm_L_base</link2></link_pair>
  <link_pair><link1>arm_L_base</link1><link2>arm_L_link1</link2></link_pair>
  <!-- 이하 모든 인접 링크 쌍에 대해 반복 -->
</disable_collisions>
```

### 위성 본체 Collision 분리

위성 본체를 본체/태양전지판/전방부로 분리하여 암과의 충돌을 세밀하게 제어:

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

---

## 관절 파라미터 튜닝

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

---

## 모델 목록 (37개)

### 위성 / 우주

| 모델 | 설명 |
|------|------|
| nasa_satellite | NASA 위성 기본 |
| nasa_satellite2~6 | NASA 위성 변형 (편대비행용) |
| gaim_target_3u | GAiMSat Target 3U 큐브샛 |
| intel_sat_dummy | Target 위성 더미 |
| capsule | 도킹 캡슐 |
| iss | ISS 모델 |
| earth | 지구 모델 |
| saturn | 토성 모델 |

### 로봇암 / 위성본체

| 모델 | 설명 |
|------|------|
| canadarm | Canadarm2 (Space ROS) |
| kari_arm | KARI 단일 암 |
| kari_arm_only | KARI 독립 암 |
| kari_dual_arm | 듀얼 암 + Chaser 위성 본체 |

### 지상 실험

| 모델 | 설명 |
|------|------|
| airbearing_satellite | 에어베어링 테스트베드 위성 |
| controla_prototype_1 | ControLA 프로토타입 1 |
| controla_prototype_2 | ControLA 프로토타입 2 |
| hagi | HAGI 모델 |
| hsr_1 | HSR 모델 |
| aruco_marker | ArUco 마커 |
| hokuyo | Hokuyo LiDAR |

### UAM

| 모델 | 설명 |
|------|------|
| uam_1 | UAM 모델 1 |
| uam_3 | UAM 모델 3 |

### 행성 / 환경

| 모델 | 설명 |
|------|------|
| lunar_surface | 달 표면 |
| martian_surface | 화성 표면 |
| moon_base | 달 기지 |
| ocean_surface | 해양 표면 (Enceladus) |
| enceladus_surface | 엔셀라두스 표면 |

### 기타

| 모델 | 설명 |
|------|------|
| nasa_ingenuity | Ingenuity 헬리콥터 |
| nasa_perseverance | Perseverance 로버 |
| curiosity_path | Curiosity 로버 경로 모델 |
| submarine | 잠수함 (Enceladus) |
| solar_panel | 태양전지판 |
| truss | 트러스 구조물 |
| X1 | X1 로버 |
| X2 | X2 로버 |

---

## World 파일 목록 (20개)

| World | 설명 |
|-------|------|
| kari_arm.sdf | 단일 KARI 암 작업 공간 |
| kari_dual_arm.sdf | 듀얼 KARI 암 작업 공간 |
| docking_world.sdf | 듀얼암 도킹 타겟 시나리오 |
| orbit.sdf | 기본 궤도 (무중력) |
| orbit2.world | 궤도 변형 |
| orbit_GEO.world | GEO 궤도 (MEV) |
| orbit_LVLH_GCO.world | LVLH 상대운동 + GCO |
| sat.world | 위성 + 센서 + 아레나 |
| gco_test.world | GCO 궤적 테스트 |
| gco_controla.world | ControLA 시나리오 |
| gco_fm_VICON.world | 편대비행 + VICON |
| gco_test_control.world | GCO 제어 입력 |
| gco_test_VICON.world | GCO + VICON 기준 |
| airbearing_testbed.world | 에어베어링 테스트베드 |
| artag_test_inc_model.world | ArUco 마커 테스트 |
| experiment.world | 범용 실험 환경 |
| moon.sdf | 달 표면 |
| mars.sdf | 화성 표면 |
| enceladus.sdf | 엔셀라두스 |
| simple.world | 최소 Gazebo world |

---

## 모델 수정 시 주의사항

- mesh 파일 경로: `package://` 대신 상대경로 `meshes/xxx.stl` 사용
- `model.sdf` + `model.config` 패턴 유지
- 새 모델 추가 시 `GZ_SIM_RESOURCE_PATH`에 경로 포함 확인
- `setup.py`의 `data_files`에 자동 등록됨 (glob 패턴)

---

*최종 업데이트: 2026-02-23*
