# check-topics

현재 실행 중인 ROS 2 시뮬레이션의 토픽/서비스/노드 상태를 점검합니다.

## 언제 사용하나

- 시뮬레이션 실행 후 통신 상태 확인할 때
- 토픽이 안 보이거나 데이터가 안 올 때 디버깅
- 새 노드 추가 후 연결 상태 검증할 때

## Steps

1. 실행 중인 노드 확인:
   ```bash
   ros2 node list
   ```

2. 토픽 목록 및 주요 토픽 상태:
   ```bash
   ros2 topic list
   ros2 topic hz /joint_states   # 주파수 확인
   ```

3. 서비스 확인:
   ```bash
   ros2 service list
   ```

4. 컨트롤러 상태 (ros2_control 사용 시):
   ```bash
   ros2 control list_controllers
   ```

5. 결과를 표 형태로 정리:
   ```
   | 노드 | 상태 | 구독 토픽 | 발행 토픽 |
   |------|------|----------|----------|
   | /impedance_controller | active | /joint_states | /effort_controller_L/commands |
   ```

6. 이상 발견 시 원인 분석 및 해결 방안 제시
