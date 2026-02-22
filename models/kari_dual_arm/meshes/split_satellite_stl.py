#!/usr/bin/env python3
"""
위성 STL 분할 스크립트
단일 STL을 body/panel/front 3개로 분할
"""

import numpy as np
from stl import mesh
import sys
import os

def split_satellite_stl(input_path, output_dir):
    """
    위성 STL을 3개로 분할
    
    분류 기준 (face center 기준):
    - body: |x| < 1.3 AND |y| < 1.3 AND |z| < 1.5
    - front: x > 1.3
    - panel: 나머지
    """
    
    print(f"Loading: {input_path}")
    sat_mesh = mesh.Mesh.from_file(input_path)
    
    n_faces = len(sat_mesh.vectors)
    print(f"Total faces: {n_faces}")
    
    # Face center 계산
    face_centers = np.mean(sat_mesh.vectors, axis=1)
    
    # 분류 기준
    body_threshold_X = 1.3
    body_threshold_Y = 1.3
    body_threshold_Z = 1.5
    
    # 분류
    is_body = (np.abs(face_centers[:, 0]) < body_threshold_X) & \
              (np.abs(face_centers[:, 1]) < body_threshold_Y) & \
              (np.abs(face_centers[:, 2]) < body_threshold_Z)
    
    is_front = face_centers[:, 0] > body_threshold_X
    
    is_panel = ~is_body & ~is_front
    
    print(f"Body faces: {np.sum(is_body)}")
    print(f"Front faces: {np.sum(is_front)}")
    print(f"Panel faces: {np.sum(is_panel)}")
    
    # 출력 디렉토리 생성
    os.makedirs(output_dir, exist_ok=True)
    
    # 각 부분 저장
    parts = [
        ('satellite_body.stl', is_body),
        ('satellite_front.stl', is_front),
        ('satellite_panel.stl', is_panel),
    ]
    
    for filename, mask in parts:
        if np.sum(mask) == 0:
            print(f"Warning: {filename} has no faces, skipping")
            continue
            
        # 해당 face만 추출
        selected_vectors = sat_mesh.vectors[mask]
        
        # 새 mesh 생성
        new_mesh = mesh.Mesh(np.zeros(len(selected_vectors), dtype=mesh.Mesh.dtype))
        new_mesh.vectors = selected_vectors
        
        # 저장
        output_path = os.path.join(output_dir, filename)
        new_mesh.save(output_path)
        print(f"Saved: {output_path} ({len(selected_vectors)} faces)")
    
    print("\n분할 완료!")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 split_satellite_stl.py <input.stl> [output_dir]")
        print("Example: python3 split_satellite_stl.py Chaser_v2_converted.stl ./meshes/")
        sys.exit(1)
    
    input_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(input_path) or '.'
    
    if not os.path.exists(input_path):
        print(f"Error: File not found: {input_path}")
        sys.exit(1)
    
    split_satellite_stl(input_path, output_dir)
