import rclpy
from rclpy.node import Node
import subprocess
import time

class TorquePublisherNode(Node):
    def __init__(self):
        super().__init__('torque_publisher_node')
        self.get_logger().info('Torque Publisher Node has started.')

        # --- 설정값 ---
        self.duration = 1.0  # 토크를 가할 시간 (초)
        self.frequency = 100 # 초당 명령 전송 횟수 (Hz)
        self.world_name = 'space_world' # .world 파일에 명시된 월드 이름
        self.target_entity = 'base_link' # 토크를 받을 링크
        self.torque_y = 500.0 # Y축 토크 값
        # --- 설정 끝 ---

        # Gazebo에 보낼 명령어 생성
        self.payload = f"""
        entity: {{ name: '{self.target_entity}', type: LINK }},
        wrench: {{ torque: {{ y: {self.torque_y} }} }}
        """
        self.command = [
            'gz', 'topic', '-t', f'/world/{self.world_name}/apply_wrench',
            '-m', 'gz.msgs.EntityWrench', '-p', self.payload
        ]

        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        self.start_time = time.time()
        
    def timer_callback(self):
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < self.duration:
            # 설정된 시간 동안 Gazebo 명령어를 계속 실행
            subprocess.run(self.command, capture_output=True, text=True)
        else:
            # 1초가 지나면 타이머를 멈추고 노드를 종료
            self.get_logger().info(f'{self.duration} second(s) have passed. Stopping torque.')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TorquePublisherNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()