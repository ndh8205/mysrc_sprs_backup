# update-env

WSL2 개발 환경 상태를 확인하고 CLAUDE.md의 환경 섹션을 업데이트합니다.

## Steps

1. 시스템 정보 수집:
   - `df -h /home/ndh/` — WSL2 디스크 잔여 용량
   - `free -h` — 메모리 사용량
   - `cat /proc/version` — WSL 커널 버전
   - `glxinfo | grep "OpenGL renderer"` — GPU 렌더링 상태
2. ROS/Gazebo 상태 확인:
   - `ros2 --version` / `printenv ROS_DISTRO`
   - `gz sim --version`
   - `echo $GZ_SIM_RESOURCE_PATH`
3. CLAUDE.md "개발 환경" 테이블에서 변경된 값만 업데이트
4. 결과를 사용자에게 보고
