# build-test

orbit_sim 패키지를 빌드하고 결과를 보고합니다.

## Steps

1. 워크스페이스 경로 확인: `~/space_ros_ws`
2. 빌드 실행:
   ```bash
   cd ~/space_ros_ws && colcon build --symlink-install --packages-select orbit_sim
   ```
3. 결과 보고:
   - 성공 시: 빌드 시간, 경고 요약
   - 실패 시: 에러 메시지 추출 및 원인 분석
4. 환경 소싱:
   ```bash
   source ~/space_ros_ws/install/setup.bash
   ```
5. 선택적 검증 (인자에 따라):
   - `ros2 pkg list | grep orbit_sim` — 패키지 등록 확인
   - `ros2 run orbit_sim --help` — 노드 실행 가능 여부
