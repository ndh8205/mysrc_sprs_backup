#!/usr/bin/env python3
"""
http_ros_bridge.py
HTTP Server for MATLAB → ROS2 communication

Endpoints:
    POST /target_pose_L  - Left arm target
    POST /target_pose_R  - Right arm target
    GET  /status         - Health check

Usage:
    ros2 run orbit_sim http_ros_bridge

Author: Space Challenge Project
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import threading
import urllib.parse


class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('http_ros_bridge')
        
        self.pub_L = self.create_publisher(PoseStamped, '/target_pose_L', 10)
        self.pub_R = self.create_publisher(PoseStamped, '/target_pose_R', 10)
        
        self.get_logger().info('HTTP-ROS2 Bridge 초기화 완료')
    
    def publish_pose(self, publisher, pos, quat):
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = float(quat[0])
        msg.pose.orientation.x = float(quat[1])
        msg.pose.orientation.y = float(quat[2])
        msg.pose.orientation.z = float(quat[3])
        publisher.publish(msg)
        return True


# Global ROS2 node reference
ros_node = None


class HTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # 로그 간소화
        pass
    
    def _set_headers(self, code=200):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
    
    def do_GET(self):
        """GET 요청 처리 (상태 확인 및 간단한 명령)"""
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)
        
        if path == '/status':
            self._set_headers(200)
            response = {'status': 'ok', 'node': 'http_ros_bridge'}
            self.wfile.write(json.dumps(response).encode())
        
        elif path == '/target_pose_L':
            # GET 방식: /target_pose_L?x=2.5&y=0.15&z=0&qw=0.7071&qx=-0.7071&qy=0&qz=0
            try:
                pos = [float(query.get('x', [0])[0]),
                       float(query.get('y', [0])[0]),
                       float(query.get('z', [0])[0])]
                quat = [float(query.get('qw', [1])[0]),
                        float(query.get('qx', [0])[0]),
                        float(query.get('qy', [0])[0]),
                        float(query.get('qz', [0])[0])]
                
                ros_node.publish_pose(ros_node.pub_L, pos, quat)
                
                self._set_headers(200)
                response = {'status': 'ok', 'arm': 'L', 'pos': pos, 'quat': quat}
                self.wfile.write(json.dumps(response).encode())
                ros_node.get_logger().info(f'L 목표: pos={pos}')
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({'error': str(e)}).encode())
        
        elif path == '/target_pose_R':
            try:
                pos = [float(query.get('x', [0])[0]),
                       float(query.get('y', [0])[0]),
                       float(query.get('z', [0])[0])]
                quat = [float(query.get('qw', [1])[0]),
                        float(query.get('qx', [0])[0]),
                        float(query.get('qy', [0])[0]),
                        float(query.get('qz', [0])[0])]
                
                ros_node.publish_pose(ros_node.pub_R, pos, quat)
                
                self._set_headers(200)
                response = {'status': 'ok', 'arm': 'R', 'pos': pos, 'quat': quat}
                self.wfile.write(json.dumps(response).encode())
                ros_node.get_logger().info(f'R 목표: pos={pos}')
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({'error': str(e)}).encode())
        
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({'error': 'Not found'}).encode())
    
    def do_POST(self):
        """POST 요청 처리 (JSON body)"""
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode('utf-8')
        
        try:
            data = json.loads(body) if body else {}
        except:
            data = {}
        
        if self.path == '/target_pose_L':
            try:
                pos = data.get('pos', [0, 0, 0])
                quat = data.get('quat', [1, 0, 0, 0])
                ros_node.publish_pose(ros_node.pub_L, pos, quat)
                
                self._set_headers(200)
                response = {'status': 'ok', 'arm': 'L', 'pos': pos}
                self.wfile.write(json.dumps(response).encode())
                ros_node.get_logger().info(f'L 목표: pos={pos}')
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({'error': str(e)}).encode())
        
        elif self.path == '/target_pose_R':
            try:
                pos = data.get('pos', [0, 0, 0])
                quat = data.get('quat', [1, 0, 0, 0])
                ros_node.publish_pose(ros_node.pub_R, pos, quat)
                
                self._set_headers(200)
                response = {'status': 'ok', 'arm': 'R', 'pos': pos}
                self.wfile.write(json.dumps(response).encode())
                ros_node.get_logger().info(f'R 목표: pos={pos}')
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({'error': str(e)}).encode())
        
        elif self.path == '/target_both':
            try:
                pos_L = data.get('pos_L', [0, 0, 0])
                pos_R = data.get('pos_R', [0, 0, 0])
                quat_L = data.get('quat_L', [1, 0, 0, 0])
                quat_R = data.get('quat_R', [1, 0, 0, 0])
                
                ros_node.publish_pose(ros_node.pub_L, pos_L, quat_L)
                ros_node.publish_pose(ros_node.pub_R, pos_R, quat_R)
                
                self._set_headers(200)
                response = {'status': 'ok', 'pos_L': pos_L, 'pos_R': pos_R}
                self.wfile.write(json.dumps(response).encode())
                ros_node.get_logger().info(f'양팔 목표: L={pos_L}, R={pos_R}')
            except Exception as e:
                self._set_headers(400)
                self.wfile.write(json.dumps({'error': str(e)}).encode())
        
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({'error': 'Not found'}).encode())


def run_http_server(port=5000):
    server = HTTPServer(('0.0.0.0', port), HTTPHandler)
    print(f'HTTP 서버 시작: http://0.0.0.0:{port}')
    server.serve_forever()


def main(args=None):
    global ros_node
    
    rclpy.init(args=args)
    ros_node = ROS2Bridge()
    
    # HTTP 서버를 별도 스레드에서 실행
    http_thread = threading.Thread(target=run_http_server, args=(5000,), daemon=True)
    http_thread.start()
    
    ros_node.get_logger().info('HTTP Bridge 서버 포트 5000에서 대기 중')
    ros_node.get_logger().info('MATLAB에서 webread/webwrite 사용 가능')
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
