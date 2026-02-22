#!/usr/bin/env python3
"""토크 인가 + 데이터 로깅 통합"""
import subprocess
import threading
import time
import math
import re
import csv
from datetime import datetime

# 전역 변수
joint_output = ""
pose_output = ""

def log_joint():
    global joint_output
    proc = subprocess.run(
        ['timeout', '7', 'gz', 'topic', '-e', '-t', 
         '/world/kari_arm_world/model/kari_arm/joint_state'],
        capture_output=True, text=True
    )
    joint_output = proc.stdout

def log_pose():
    global pose_output
    proc = subprocess.run(
        ['timeout', '7', 'gz', 'topic', '-e', '-t',
         '/world/kari_arm_world/pose/info'],
        capture_output=True, text=True
    )
    pose_output = proc.stdout

def send_torque(torque):
    subprocess.run(['gz', 'topic', '-t', '/model/kari_arm/joint/joint1/cmd_force',
                   '-m', 'gz.msgs.Double', '-p', f'data: {torque}'],
                  capture_output=True)

def run_torque():
    time.sleep(1)  # 로깅 시작 대기
    print("토크 인가 시작...")
    
    duration = 5.0
    dt = 0.01
    t = 0.0
    start = time.time()
    
    while t < duration:
        torque = 0.1 * math.sin(t)
        send_torque(torque)
        
        if int(t*10) % 10 == 0:
            print(f"  t={t:.1f}s, tau={torque:+.4f}")
        
        t += dt
        target = start + t
        if target > time.time():
            time.sleep(target - time.time())
    
    send_torque(0.0)
    print("토크 인가 완료!")

# 스레드 시작
print("=" * 50)
print("통합 테스트: 토크 + 로깅")
print("=" * 50)

t1 = threading.Thread(target=log_joint)
t2 = threading.Thread(target=log_pose)
t3 = threading.Thread(target=run_torque)

t1.start()
t2.start()
t3.start()

t3.join()
print("로깅 대기 중...")
t1.join()
t2.join()

print("파싱 중...")

# joint 파싱
joint_data = []
blocks = re.split(r'(?=joint \{)', joint_output)
for block in blocks:
    positions = re.findall(r'position:\s*([-\d.e+]+)', block)
    velocities = re.findall(r'velocity:\s*([-\d.e+]+)', block)
    if len(positions) >= 7:
        joint_data.append({
            'pos': [float(p) for p in positions[:7]],
            'vel': [float(v) for v in velocities[:7]] if len(velocities) >= 7 else [0]*7
        })

# pose 파싱
pose_data = []
pattern = r'name:\s*"satellite_body".*?position\s*\{([^}]+)\}.*?orientation\s*\{([^}]+)\}'
matches = re.findall(pattern, pose_output, re.DOTALL)
for pos_str, ori_str in matches:
    try:
        x = float(re.search(r'x:\s*([-\d.e+]+)', pos_str).group(1))
        y = float(re.search(r'y:\s*([-\d.e+]+)', pos_str).group(1))
        z = float(re.search(r'z:\s*([-\d.e+]+)', pos_str).group(1))
        qx = float(re.search(r'x:\s*([-\d.e+]+)', ori_str).group(1))
        qy = float(re.search(r'y:\s*([-\d.e+]+)', ori_str).group(1))
        qz = float(re.search(r'z:\s*([-\d.e+]+)', ori_str).group(1))
        qw = float(re.search(r'w:\s*([-\d.e+]+)', ori_str).group(1))
        pose_data.append({'pos': [x,y,z], 'quat': [qw,qx,qy,qz]})
    except:
        pass

print(f"Joint 샘플: {len(joint_data)}, Pose 샘플: {len(pose_data)}")

n = min(len(joint_data), len(pose_data))
if n == 0:
    print("데이터 없음! Gazebo 실행 중인지 확인")
    exit(1)

dt = 5.0 / n
filename = f"gazebo_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

with open(filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['time','torque','q1','q2','q3','q4','q5','q6','q7',
                     'qd1','qd2','qd3','qd4','qd5','qd6','qd7',
                     'sat_x','sat_y','sat_z','sat_qw','sat_qx','sat_qy','sat_qz'])
    
    for i in range(n):
        t = i * dt
        tau = 0.1 * math.sin(t)
        row = [t, tau] + joint_data[i]['pos'] + joint_data[i]['vel'] + \
              pose_data[i]['pos'] + pose_data[i]['quat']
        writer.writerow(row)

print(f"저장 완료: {filename}")
print(f"MATLAB: data = readtable('{filename}');")