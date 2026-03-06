#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

import tkinter as tk
from tkinter import ttk
import threading
import math
import os
import json
from datetime import datetime

import numpy as np
from cv_bridge import CvBridge
import cv2


# Stereo camera spec (WFOV, 62deg)
CAM_FOV = 1.082       # rad
CAM_WIDTH = 1024       # px
CAM_BASELINE = 0.07    # m (70mm)
CAM_F_PX = (CAM_WIDTH / 2.0) / math.tan(CAM_FOV / 2.0)  # ~1755

STEREO_TOPICS = {
    'left':  '/stereo/left/image_raw',
    'right': '/stereo/right/image_raw',
}


class StereoTestGui(Node):
    def __init__(self, gui_root):
        super().__init__('stereo_test_gui')

        self.gui_root = gui_root
        self.bridge = CvBridge()

        # Image buffers
        self.image_buffers = {key: None for key in STEREO_TOPICS}

        # Subscribe stereo pair
        self.subscribers = {}
        for key, topic in STEREO_TOPICS.items():
            self.subscribers[key] = self.create_subscription(
                Image, topic,
                lambda msg, k=key: self._image_callback(k, msg),
                1
            )

        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/model/gaimsat_target/odometry',
            self._odom_callback, 10
        )

        # SetEntityPose
        self.pose_client = self.create_client(
            SetEntityPose,
            '/world/stereo_test_world/set_pose'
        )

        self.output_base = os.path.expanduser('~/stereo_captures')

        # Deputy pose (moves along -X axis, cameras face +X toward origin)
        self.dep_x = tk.DoubleVar(value=-150.0)
        self.dep_y = tk.DoubleVar(value=0.0)
        self.dep_z = tk.DoubleVar(value=0.0)
        self.dep_roll = tk.DoubleVar(value=0.0)
        self.dep_pitch = tk.DoubleVar(value=0.0)
        self.dep_yaw = tk.DoubleVar(value=0.0)

        # Target rotation
        self.tgt_roll = tk.DoubleVar(value=0.0)
        self.tgt_pitch = tk.DoubleVar(value=0.0)
        self.tgt_yaw = tk.DoubleVar(value=0.0)

        # Status
        self.cur_x = tk.StringVar(value='0.000')
        self.cur_y = tk.StringVar(value='0.000')
        self.cur_z = tk.StringVar(value='0.000')
        self.cur_dist = tk.StringVar(value='0.000')
        self.capture_status = tk.StringVar(value='Ready')
        self.buf_left = tk.StringVar(value='--')
        self.buf_right = tk.StringVar(value='--')
        self.disp_val = tk.StringVar(value='-')

        self._setup_gui()

        self.get_logger().info('Waiting for SetEntityPose service...')
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Still waiting...')
        self.get_logger().info('Service ready.')

    def _image_callback(self, key, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_buffers[key] = cv_img
            if key == 'left':
                self.buf_left.set('OK')
            else:
                self.buf_right.set('OK')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error ({key}): {e}')

    def _odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.cur_x.set(f'{x:.3f}')
        self.cur_y.set(f'{y:.3f}')
        self.cur_z.set(f'{z:.3f}')
        dist = math.sqrt(x*x + y*y + z*z)
        self.cur_dist.set(f'{dist:.2f}')
        self._update_disparity(dist)

    def _update_disparity(self, dist_m):
        if dist_m < 0.01:
            dist_m = 0.01
        d = CAM_F_PX * CAM_BASELINE / dist_m
        self.disp_val.set(f'{d:.2f} px')

    @staticmethod
    def _euler_to_quat(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def _send_pose(self, entity_name, x, y, z, roll, pitch, yaw):
        req = SetEntityPose.Request()
        req.entity.name = entity_name
        req.entity.type = 2
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation = self._euler_to_quat(roll, pitch, yaw)
        req.pose = pose
        future = self.pose_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Pose: {entity_name} -> ({x:.1f},{y:.1f},{z:.1f})')
        )

    def _apply_deputy_pose(self):
        self._send_pose(
            'deputy_gaimsat',
            self.dep_x.get(), self.dep_y.get(), self.dep_z.get(),
            self.dep_roll.get(), self.dep_pitch.get(), self.dep_yaw.get(),
        )

    def _apply_target_rotation(self):
        self._send_pose(
            'chief_target',
            0.0, 0.0, 0.0,
            self.tgt_roll.get(), self.tgt_pitch.get(), self.tgt_yaw.get(),
        )

    def _apply_preset(self, x_val):
        self.dep_x.set(x_val)
        self.dep_y.set(0.0)
        self.dep_z.set(0.0)
        self._apply_deputy_pose()

    def _capture(self):
        empty = [k for k, v in self.image_buffers.items() if v is None]
        if empty:
            self.capture_status.set(f'Missing: {", ".join(empty)}')
            self.get_logger().warn(f'Capture skipped — no data: {empty}')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.join(self.output_base, timestamp)
        os.makedirs(out_dir, exist_ok=True)

        for key, img in self.image_buffers.items():
            cv2.imwrite(os.path.join(out_dir, f'{key}.png'), img)

        dist_m = float(self.cur_dist.get())
        if dist_m < 0.01:
            dist_m = 0.01
        metadata = {
            'timestamp': timestamp,
            'deputy_pose': {
                'x': self.dep_x.get(), 'y': self.dep_y.get(), 'z': self.dep_z.get(),
                'roll': self.dep_roll.get(), 'pitch': self.dep_pitch.get(), 'yaw': self.dep_yaw.get(),
            },
            'target_rotation': {
                'roll': self.tgt_roll.get(), 'pitch': self.tgt_pitch.get(), 'yaw': self.tgt_yaw.get(),
            },
            'distance_m': dist_m,
            'camera': {
                'fov_rad': CAM_FOV,
                'f_px': round(CAM_F_PX, 2),
                'baseline_m': CAM_BASELINE,
                'resolution': f'{CAM_WIDTH}x{CAM_WIDTH}',
                'expected_disparity_px': round(CAM_F_PX * CAM_BASELINE / dist_m, 4),
            },
        }
        with open(os.path.join(out_dir, 'metadata.json'), 'w') as f:
            json.dump(metadata, f, indent=2)

        self.capture_status.set(f'Saved: {timestamp}')
        self.get_logger().info(f'Captured → {out_dir}')

    def _shutdown(self):
        self.get_logger().info('Shutting down.')
        self.destroy_node()
        self.gui_root.quit()
        rclpy.shutdown()

    def _setup_gui(self):
        self.gui_root.title('Stereo Test — GAIMSat-1')
        main = ttk.Frame(self.gui_root, padding='10')
        main.grid(row=0, column=0, sticky='nsew')
        left = ttk.Frame(main)
        left.grid(row=0, column=0, padx=5, sticky='nsew')
        right = ttk.Frame(main)
        right.grid(row=0, column=1, padx=5, sticky='nsew')

        # ---- Left: Controls ----
        ttk.Label(left, text='STEREO TEST',
                  font=('Arial', 14, 'bold')).grid(row=0, column=0, columnspan=2, pady=5)

        # Distance presets (Deputy moves along -X)
        pf = ttk.LabelFrame(left, text='Distance Presets', padding='8')
        pf.grid(row=1, column=0, columnspan=2, sticky='ew', pady=3)
        presets = [
            ('5 km', -5000), ('1 km', -1000), ('150 m', -150),
            ('20 m', -20), ('2 m', -2),
        ]
        for i, (label, val) in enumerate(presets):
            ttk.Button(pf, text=label, width=8,
                       command=lambda v=val: self._apply_preset(v)).grid(row=0, column=i, padx=2)

        # Deputy pose
        df = ttk.LabelFrame(left, text='Deputy Pose', padding='8')
        df.grid(row=2, column=0, columnspan=2, sticky='ew', pady=3)
        for i, (lbl, var) in enumerate([
            ('X:', self.dep_x), ('Y:', self.dep_y), ('Z:', self.dep_z),
            ('Roll:', self.dep_roll), ('Pitch:', self.dep_pitch), ('Yaw:', self.dep_yaw),
        ]):
            ttk.Label(df, text=lbl, width=6).grid(row=i, column=0, sticky='w')
            ttk.Entry(df, textvariable=var, width=12).grid(row=i, column=1, padx=3)
        ttk.Button(df, text='Apply Deputy', command=self._apply_deputy_pose).grid(
            row=6, column=0, columnspan=2, pady=5)

        # Target rotation
        tf = ttk.LabelFrame(left, text='Target Rotation', padding='8')
        tf.grid(row=3, column=0, columnspan=2, sticky='ew', pady=3)
        for i, (lbl, var) in enumerate([
            ('Roll:', self.tgt_roll), ('Pitch:', self.tgt_pitch), ('Yaw:', self.tgt_yaw),
        ]):
            ttk.Label(tf, text=lbl, width=6).grid(row=i, column=0, sticky='w')
            ttk.Entry(tf, textvariable=var, width=12).grid(row=i, column=1, padx=3)
        ttk.Button(tf, text='Apply Target', command=self._apply_target_rotation).grid(
            row=3, column=0, columnspan=2, pady=5)

        # Capture + Quit
        bf = ttk.Frame(left, padding='5')
        bf.grid(row=4, column=0, columnspan=2, sticky='ew', pady=5)
        ttk.Button(bf, text='  CAPTURE  ', command=self._capture).pack(side='left', padx=5)
        ttk.Label(bf, textvariable=self.capture_status, font=('Courier', 9)).pack(side='left', padx=5)
        ttk.Button(left, text='Quit', command=self._shutdown).grid(
            row=5, column=0, columnspan=2, pady=5)

        # ---- Right: Status ----
        ttk.Label(right, text='STATUS',
                  font=('Arial', 14, 'bold')).grid(row=0, column=0, pady=5)

        sf = ttk.LabelFrame(right, text='Deputy Position', padding='8')
        sf.grid(row=1, column=0, sticky='ew', pady=3)
        for i, (lbl, var) in enumerate([
            ('X:', self.cur_x), ('Y:', self.cur_y), ('Z:', self.cur_z),
            ('Dist:', self.cur_dist),
        ]):
            ttk.Label(sf, text=lbl, width=6).grid(row=i, column=0, sticky='w')
            ttk.Label(sf, textvariable=var, width=12, relief='sunken',
                      anchor='e', font=('Courier', 10)).grid(row=i, column=1, padx=3)

        ef = ttk.LabelFrame(right, text='Expected Disparity', padding='8')
        ef.grid(row=2, column=0, sticky='ew', pady=3)
        ttk.Label(ef, text='d =', width=4).grid(row=0, column=0, sticky='w')
        ttk.Label(ef, textvariable=self.disp_val, width=12, relief='sunken',
                  anchor='e', font=('Courier', 10)).grid(row=0, column=1, padx=3)
        ttk.Label(ef, text='f*B/Z', font=('Arial', 8, 'italic')).grid(row=0, column=2, padx=3)

        bf2 = ttk.LabelFrame(right, text='Image Buffers', padding='8')
        bf2.grid(row=3, column=0, sticky='ew', pady=3)
        ttk.Label(bf2, text='Left:').grid(row=0, column=0, sticky='w')
        ttk.Label(bf2, textvariable=self.buf_left, width=6, relief='sunken',
                  anchor='center', font=('Courier', 9)).grid(row=0, column=1)
        ttk.Label(bf2, text='Right:').grid(row=1, column=0, sticky='w')
        ttk.Label(bf2, textvariable=self.buf_right, width=6, relief='sunken',
                  anchor='center', font=('Courier', 9)).grid(row=1, column=1)

        info = ttk.LabelFrame(right, text='Camera Spec', padding='8')
        info.grid(row=4, column=0, sticky='ew', pady=3)
        ttk.Label(info, text=(
            f'FOV: 62° (1.082 rad)\n'
            f'f_px: {CAM_F_PX:.0f}\n'
            f'Baseline: 70mm\n'
            f'Res: 1024x1024 @ 5Hz\n'
            f'Mount: +X face, no rotation'
        ), font=('Courier', 9), justify='left').grid(row=0, column=0, padx=3)


def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    node = StereoTestGui(root)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    root.mainloop()


if __name__ == '__main__':
    main()
