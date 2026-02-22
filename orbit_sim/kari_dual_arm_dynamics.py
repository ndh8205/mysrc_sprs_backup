#!/usr/bin/env python3
"""
kari_dual_arm_dynamics.py
KARI Dual ARM 동역학 라이브러리 (Free-floating Space Manipulator)

Reference: Zong et al., "Reactionless Control of Free-floating Space Manipulators"
           IEEE TAES, 2019

Author: Space Challenge Project
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional


# ============================================================
#  파라미터 정의 (SDF/URDF 기반)
# ============================================================

@dataclass
class LinkParams:
    """링크 파라미터"""
    name: str
    mass: float
    com: np.ndarray      # CoM in link frame [m]
    inertia: np.ndarray  # 3x3 inertia tensor [kg*m^2]
    joint_type: str      # 'revolute' or 'fixed'
    joint_axis: np.ndarray
    joint_offset: np.ndarray  # xyz offset from parent


class KariArmParams:
    """KARI ARM 시스템 파라미터 (단일 팔)"""
    
    def __init__(self, arm_id: str = 'L'):
        """
        Args:
            arm_id: 'L' (Left, +Y) or 'R' (Right, -Y)
        """
        self.arm_id = arm_id
        
        # ===== 위성 파라미터 =====
        self.sat_mass = 500.0  # [kg]
        self.sat_inertia = np.array([
            [260.0, -0.2, 0.6],
            [-0.2, 280.0, 4.0],
            [0.6, 4.0, 170.0]
        ])
        
        # 장착점 및 회전 (위성 body frame 기준)
        if arm_id == 'L':
            # Left arm (+Y): X=1.9, Y=0.9, Rx=-90°
            self.r_mount = np.array([1.9, 0.9, 0.0])
            self.R_mount = self._rot_x(-np.pi/2)
        else:
            # Right arm (-Y): X=1.9, Y=-0.9, Rx=+90°
            self.r_mount = np.array([1.9, -0.9, 0.0])
            self.R_mount = self._rot_x(np.pi/2)
        
        # ===== 로봇팔 파라미터 (7-DOF) =====
        self.n_joints = 7
        self.n_bodies = 9  # base + link1~7 + eef
        
        # Joint offsets from parent (URDF 기준)
        self.joint_offsets = [
            np.array([0.0, 0.0, 0.0]),       # base (from mount)
            np.array([0.0, 0.0, 0.094]),     # joint1
            np.array([0.0, 0.088, 0.105]),   # joint2
            np.array([0.0, 0.104, 0.131]),   # joint3
            np.array([0.0, -0.088, 0.8]),    # joint4
            np.array([0.0, -0.104, 0.131]),  # joint5
            np.array([0.0, -0.071, 0.408]),  # joint6
            np.array([0.0, -0.121, 0.088]),  # joint7
            np.array([0.0, 0.0, 0.097]),     # eef
        ]
        
        # Joint axes (local frame)
        self.joint_axes = [
            np.array([0, 0, 1]),  # joint1 - Z
            np.array([0, 1, 0]),  # joint2 - Y
            np.array([0, 0, 1]),  # joint3 - Z
            np.array([0, 1, 0]),  # joint4 - Y
            np.array([0, 0, 1]),  # joint5 - Z
            np.array([0, 1, 0]),  # joint6 - Y
            np.array([0, 0, 1]),  # joint7 - Z
        ]
        
        # Link masses [kg]
        self.link_masses = [
            3.3436,   # base
            2.1207,   # link1
            6.6621,   # link2
            4.0005,   # link3
            6.66208,  # link4
            2.6161,   # link5
            4.9702,   # link6
            3.3715,   # link7
            0.001,    # eef
        ]
        
        # Link CoM positions (link local frame) [m]
        self.link_coms = [
            np.array([-0.00000965, -0.00003686, 0.07767998]),  # base
            np.array([-0.00026892, -0.08021331, 0.09358186]),  # link1
            np.array([0.0, 0.06124762, 0.05639568]),           # link2
            np.array([-0.00033293, -0.00482569, 0.40191520]),  # link3
            np.array([0.0, -0.06124762, 0.05639568]),          # link4
            np.array([-0.00050908, -0.00461248, 0.21393427]),  # link5
            np.array([0.00011474, -0.05246547, -0.03422529]),  # link6
            np.array([0.00000590, 0.0, 0.01747537]),           # link7
            np.array([0.0, 0.0, 0.0]),                          # eef
        ]
        
        # Link inertia tensors (link CoM frame) [kg*m^2]
        self.link_inertias = [
            np.array([[0.00751250, -0.00000006, 0.00000167],
                      [-0.00000006, 0.00752435, -0.00000111],
                      [0.00000167, -0.00000111, 0.00631252]]),  # base
            np.array([[0.02665777, 0.00012951, -0.00002113],
                      [0.00012951, 0.00821363, -0.00194208],
                      [-0.00002113, -0.00194208, 0.02512228]]),  # link1
            np.array([[0.05097328, 0.00000002, 0.0],
                      [0.00000002, 0.03629437, 0.01622322],
                      [0.0, 0.01622322, 0.02722892]]),           # link2
            np.array([[0.29888290, -0.00000478, 0.00033497],
                      [-0.00000478, 0.29796389, -0.00781668],
                      [0.00033497, -0.00781668, 0.01207673]]),   # link3
            np.array([[0.05097328, 0.00000002, 0.0],
                      [0.00000002, 0.03629437, -0.01622322],
                      [0.0, -0.01622322, 0.02722892]]),          # link4
            np.array([[0.05207280, -0.00000483, 0.00009635],
                      [-0.00000483, 0.05170972, -0.00247387],
                      [0.00009635, -0.00247387, 0.00805695]]),   # link5
            np.array([[0.05197714, -0.00005372, -0.00015573],
                      [-0.00005372, 0.03740379, 0.01165841],
                      [-0.00015573, 0.01165841, 0.02571022]]),   # link6
            np.array([[0.00768334, 0.0, 0.00000006],
                      [0.0, 0.00768346, 0.0],
                      [0.00000006, 0.0, 0.00607932]]),           # link7
            np.array([[0.000001, 0, 0],
                      [0, 0.000001, 0],
                      [0, 0, 0.000001]]),                        # eef
        ]
        
        # Joint types
        self.joint_types = ['fixed', 'revolute', 'revolute', 'revolute', 
                           'revolute', 'revolute', 'revolute', 'revolute', 'fixed']
        
        # ===== 모터 파라미터 =====
        self.motor_tau_max = np.array([65, 65, 50, 50, 30, 30, 20], dtype=float)
        self.motor_gear_ratio = np.array([160, 160, 120, 120, 80, 80, 60], dtype=float)
        self.motor_I_motor = np.array([0.0005, 0.0005, 0.0004, 0.0004, 0.0003, 0.0003, 0.0002])
        
        I_reflected = self.motor_I_motor * self.motor_gear_ratio**2
        tau_base = 0.05
        I_ref = 5.0
        self.motor_tau_up = tau_base * np.sqrt(I_reflected / I_ref)
        self.motor_tau_down = self.motor_tau_up * 0.4
    
    @staticmethod
    def _rot_x(angle: float) -> np.ndarray:
        """X축 회전행렬"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])


# ============================================================
#  유틸리티 함수
# ============================================================

def skew3(v: np.ndarray) -> np.ndarray:
    """3D 벡터의 skew-symmetric 행렬"""
    v = v.flatten()
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """쿼터니언 → DCM (Hamilton convention: [qw, qx, qy, qz])"""
    q = q.flatten()
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    return R


def dcm_to_quat(R: np.ndarray) -> np.ndarray:
    """DCM → 쿼터니언 (Shepperd method)"""
    trace_R = np.trace(R)
    
    if trace_R > 0:
        s = 0.5 / np.sqrt(trace_R + 1)
        q = np.array([0.25/s, (R[2,1] - R[1,2])*s, (R[0,2] - R[2,0])*s, (R[1,0] - R[0,1])*s])
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2 * np.sqrt(1 + R[0,0] - R[1,1] - R[2,2])
        q = np.array([(R[2,1] - R[1,2])/s, 0.25*s, (R[0,1] + R[1,0])/s, (R[0,2] + R[2,0])/s])
    elif R[1,1] > R[2,2]:
        s = 2 * np.sqrt(1 + R[1,1] - R[0,0] - R[2,2])
        q = np.array([(R[0,2] - R[2,0])/s, (R[0,1] + R[1,0])/s, 0.25*s, (R[1,2] + R[2,1])/s])
    else:
        s = 2 * np.sqrt(1 + R[2,2] - R[0,0] - R[1,1])
        q = np.array([(R[1,0] - R[0,1])/s, (R[0,2] + R[2,0])/s, (R[1,2] + R[2,1])/s, 0.25*s])
    
    return q / np.linalg.norm(q)


def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """쿼터니언 곱셈 (Hamilton convention)"""
    q1, q2 = q1.flatten(), q2.flatten()
    a, b, c, d = q1[0], q1[1], q1[2], q1[3]
    e, f, g, h = q2[0], q2[1], q2[2], q2[3]
    
    return np.array([
        a*e - b*f - c*g - d*h,
        a*f + b*e + c*h - d*g,
        a*g - b*h + c*e + d*f,
        a*h + b*g - c*f + d*e
    ])


def quat_conj(q: np.ndarray) -> np.ndarray:
    """쿼터니언 켤레"""
    q = q.flatten()
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_inv(q: np.ndarray) -> np.ndarray:
    """쿼터니언 역"""
    return quat_conj(q) / np.linalg.norm(q)**2


def axis_angle_to_dcm(axis: np.ndarray, angle: float) -> np.ndarray:
    """Axis-angle → DCM (Rodrigues' formula)"""
    axis = axis.flatten()
    axis = axis / np.linalg.norm(axis)
    
    if abs(angle) < 1e-10:
        return np.eye(3)
    
    K = skew3(axis)
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return R


# ============================================================
#  Forward Kinematics
# ============================================================

@dataclass
class FKResult:
    """FK 결과 구조체"""
    R_base: np.ndarray          # 위성 회전행렬
    p_base: np.ndarray          # 위성 CoM 위치
    p_mount: np.ndarray         # 장착점 위치
    R_mount: np.ndarray         # 장착점 회전행렬 (관성좌표계)
    R: List[np.ndarray]         # 각 링크 회전행렬 (revolute만)
    p_joint: List[np.ndarray]   # 각 joint 위치 (revolute만)
    p_com: List[np.ndarray]     # 각 링크 CoM 위치 (revolute만)
    k: List[np.ndarray]         # 각 joint 회전축 (관성좌표계)
    R_all: List[np.ndarray]     # 모든 body 회전행렬
    p_com_all: List[np.ndarray] # 모든 body CoM 위치
    p_ee: np.ndarray            # End-effector 위치
    R_ee: np.ndarray            # End-effector 자세


def forward_kinematics(params: KariArmParams, 
                       p_base: np.ndarray, 
                       q_base: np.ndarray, 
                       theta: np.ndarray) -> FKResult:
    """
    Forward Kinematics 계산 (장착 회전 적용)
    
    Args:
        params: 로봇 파라미터
        p_base: 위성 CoM 위치 [3,]
        q_base: 위성 자세 쿼터니언 [qw, qx, qy, qz]
        theta: 관절 각도 [7,]
    
    Returns:
        FKResult 구조체
    """
    p_base = p_base.flatten()
    q_base = q_base.flatten()
    theta = theta.flatten()
    
    # 위성 회전행렬
    R_base = quat_to_dcm(q_base)
    
    # 장착점 위치 및 회전 (관성좌표계)
    p_mount = p_base + R_base @ params.r_mount
    R_mount_I = R_base @ params.R_mount  # 장착 프레임 회전
    
    # 결과 저장용
    R_list = []
    p_joint_list = []
    p_com_list = []
    k_list = []
    R_all = []
    p_com_all = []
    
    # 현재 프레임 상태 (장착 회전 적용)
    R_current = R_mount_I
    p_current = p_mount
    
    joint_idx = 0
    
    for i in range(params.n_bodies):
        offset = params.joint_offsets[i]
        
        # Joint 위치
        p_joint_i = p_current + R_current @ offset
        
        if params.joint_types[i] == 'revolute':
            # 회전축 (관성좌표계)
            axis_local = params.joint_axes[joint_idx]
            k_i = R_current @ axis_local
            k_i = k_i / np.linalg.norm(k_i)
            
            # Joint 회전
            R_joint = axis_angle_to_dcm(axis_local, theta[joint_idx])
            R_link = R_current @ R_joint
            
            # CoM 위치
            com_local = params.link_coms[i]
            p_com_i = p_joint_i + R_link @ com_local
            
            # 저장
            R_list.append(R_link)
            p_joint_list.append(p_joint_i)
            p_com_list.append(p_com_i)
            k_list.append(k_i)
            
            R_all.append(R_link)
            p_com_all.append(p_com_i)
            
            R_current = R_link
            p_current = p_joint_i
            joint_idx += 1
            
        else:  # fixed
            R_link = R_current
            p_link = p_joint_i
            
            com_local = params.link_coms[i]
            p_com_i = p_link + R_link @ com_local
            
            R_all.append(R_link)
            p_com_all.append(p_com_i)
            
            R_current = R_link
            p_current = p_link
    
    # End-effector (마지막 프레임)
    p_ee = p_current
    R_ee = R_current
    
    return FKResult(
        R_base=R_base,
        p_base=p_base,
        p_mount=p_mount,
        R_mount=R_mount_I,
        R=R_list,
        p_joint=p_joint_list,
        p_com=p_com_list,
        k=k_list,
        R_all=R_all,
        p_com_all=p_com_all,
        p_ee=p_ee,
        R_ee=R_ee
    )


# ============================================================
#  Jacobian
# ============================================================

def compute_jacobian(fk: FKResult, params: KariArmParams) -> Tuple[np.ndarray, np.ndarray]:
    """
    Base Jacobian (Jb) 및 Manipulator Jacobian (Jm) 계산
    
    ẋ_ee = Jb * [v_base; ω_base] + Jm * θ̇
    
    Returns:
        Jb: (6, 6) Base Jacobian
        Jm: (6, n_joints) Manipulator Jacobian
    """
    n_joints = params.n_joints
    p_ee = fk.p_ee
    p_mount = fk.p_mount
    
    # Base Jacobian
    p_0e = p_ee - p_mount
    Jb = np.zeros((6, 6))
    Jb[0:3, 0:3] = np.eye(3)
    Jb[0:3, 3:6] = -skew3(p_0e)
    Jb[3:6, 3:6] = np.eye(3)
    
    # Manipulator Jacobian
    Jm = np.zeros((6, n_joints))
    for j in range(n_joints):
        k_j = fk.k[j]
        p_j = fk.p_joint[j]
        r_je = p_ee - p_j
        
        Jm[0:3, j] = np.cross(k_j, r_je)
        Jm[3:6, j] = k_j
    
    return Jb, Jm


def compute_generalized_jacobian(fk: FKResult, H: np.ndarray, 
                                  params: KariArmParams) -> np.ndarray:
    """
    Generalized Jacobian 계산 (Free-floating 특성 반영)
    
    J* = Jm - Jb * H_bb^{-1} * H_bm
    
    Args:
        fk: FK 결과
        H: Mass matrix (13x13)
        params: 로봇 파라미터
    
    Returns:
        Jg: (6, n_joints) Generalized Jacobian
    """
    Jb, Jm = compute_jacobian(fk, params)
    
    H_bb = H[0:6, 0:6]
    H_bm = H[0:6, 6:]
    
    H_bb_inv = np.linalg.inv(H_bb)
    Jg = Jm - Jb @ H_bb_inv @ H_bm
    
    return Jg


# ============================================================
#  Mass Matrix
# ============================================================

def compute_mass_matrix(fk: FKResult, params: KariArmParams) -> np.ndarray:
    """
    일반화 관성행렬 (Mass Matrix) 계산
    
    H = [M*I_3,  H_vw,   J_Tw^T]
        [H_vw^T, H_w,    H_wq  ]
        [J_Tw,   H_wq^T, H_m   ]
    
    Returns:
        H: (6+n_joints, 6+n_joints) Mass Matrix
    """
    n_joints = params.n_joints
    n_bodies = params.n_bodies
    
    # 위성 관성 (관성좌표계)
    I_sat = fk.R_base @ params.sat_inertia @ fk.R_base.T
    
    # 링크별 Jacobian 및 관성
    J_T = []  # 선속도 Jacobian
    J_R = []  # 각속도 Jacobian
    I_links = []  # 관성텐서 (관성좌표계)
    m_links = []  # 질량
    r_0i = []  # p_base에서 CoM까지 벡터
    
    joint_count = 0
    for i in range(n_bodies):
        if params.joint_types[i] != 'revolute':
            continue
            
        m = params.link_masses[i]
        R_i = fk.R_all[i]
        I_local = params.link_inertias[i]
        I_i = R_i @ I_local @ R_i.T
        
        p_com_i = fk.p_com_all[i]
        r_i = p_com_i - fk.p_base
        
        # 선속도 Jacobian
        J_Ti = np.zeros((3, n_joints))
        for j in range(joint_count + 1):
            k_j = fk.k[j]
            r_ji = p_com_i - fk.p_joint[j]
            J_Ti[:, j] = np.cross(k_j, r_ji)
        
        # 각속도 Jacobian
        J_Ri = np.zeros((3, n_joints))
        for j in range(joint_count + 1):
            J_Ri[:, j] = fk.k[j]
        
        J_T.append(J_Ti)
        J_R.append(J_Ri)
        I_links.append(I_i)
        m_links.append(m)
        r_0i.append(r_i)
        
        joint_count += 1
    
    # 전체 질량
    M = params.sat_mass + sum(m_links)
    
    # 시스템 CoM
    r_0g = np.zeros(3)
    for i in range(n_joints):
        r_0g += m_links[i] * r_0i[i]
    r_0g /= M
    
    # H_vw
    H_vw = -M * skew3(r_0g)
    
    # H_w
    H_w = I_sat.copy()
    for i in range(n_joints):
        H_w += I_links[i] + m_links[i] * skew3(r_0i[i]).T @ skew3(r_0i[i])
    
    # H_m
    H_m = np.zeros((n_joints, n_joints))
    for i in range(n_joints):
        H_m += m_links[i] * (J_T[i].T @ J_T[i]) + J_R[i].T @ I_links[i] @ J_R[i]
    
    # H_wq
    H_wq = np.zeros((3, n_joints))
    for i in range(n_joints):
        H_wq += m_links[i] * skew3(r_0i[i]) @ J_T[i] + I_links[i] @ J_R[i]
    
    # J_Tw
    J_Tw = np.zeros((3, n_joints))
    for i in range(n_joints):
        J_Tw += m_links[i] * J_T[i]
    
    # 조립
    H = np.zeros((6 + n_joints, 6 + n_joints))
    H[0:3, 0:3] = M * np.eye(3)
    H[0:3, 3:6] = H_vw
    H[0:3, 6:] = J_Tw
    H[3:6, 0:3] = H_vw.T
    H[3:6, 3:6] = H_w
    H[3:6, 6:] = H_wq
    H[6:, 0:3] = J_Tw.T
    H[6:, 3:6] = H_wq.T
    H[6:, 6:] = H_m
    
    return H


# ============================================================
#  Coriolis / Centrifugal
# ============================================================

def compute_coriolis(fk: FKResult, params: KariArmParams,
                     v_base: np.ndarray, omega_base: np.ndarray,
                     theta_dot: np.ndarray) -> np.ndarray:
    """
    Coriolis/Centrifugal 비선형 항 계산 (Recursive Newton-Euler)
    
    Returns:
        c: (6+n_joints,) Coriolis vector
    """
    n_joints = params.n_joints
    n_bodies = params.n_bodies
    
    v_base = v_base.flatten()
    omega_base = omega_base.flatten()
    theta_dot = theta_dot.flatten()
    
    # Body별 속도/가속도 저장
    w = [np.zeros(3) for _ in range(n_bodies + 1)]   # 각속도
    wd = [np.zeros(3) for _ in range(n_bodies + 1)]  # 각가속도
    vd = [np.zeros(3) for _ in range(n_bodies + 1)]  # 선가속도
    vc = [np.zeros(3) for _ in range(n_bodies)]      # CoM 가속도
    
    F = [np.zeros(3) for _ in range(n_bodies)]  # 힘
    N = [np.zeros(3) for _ in range(n_bodies)]  # 토크
    
    # Forward pass
    w[0] = omega_base
    
    joint_idx = 0
    for i in range(n_bodies):
        if i == 0:  # base link (fixed to satellite)
            w[i+1] = w[0]
            wd[i+1] = np.zeros(3)
        elif params.joint_types[i] == 'revolute':
            k = fk.k[joint_idx]
            w[i+1] = w[i] + theta_dot[joint_idx] * k
            wd[i+1] = wd[i] + np.cross(w[i], theta_dot[joint_idx] * k)
            joint_idx += 1
        else:
            w[i+1] = w[i]
            wd[i+1] = wd[i]
    
    # CoM 가속도 및 힘/토크
    joint_idx = 0
    for i in range(n_bodies):
        m_i = params.link_masses[i]
        R_i = fk.R_all[i]
        I_i = R_i @ params.link_inertias[i] @ R_i.T
        p_com_i = fk.p_com_all[i]
        
        # 원심 가속도
        if i == 0:
            r = p_com_i - fk.p_mount
        else:
            r = p_com_i - fk.p_joint[min(joint_idx, n_joints-1)]
        
        vc[i] = np.cross(wd[i+1], r) + np.cross(w[i+1], np.cross(w[i+1], r))
        
        F[i] = m_i * vc[i]
        N[i] = I_i @ wd[i+1] + np.cross(w[i+1], I_i @ w[i+1])
        
        if params.joint_types[i] == 'revolute':
            joint_idx += 1
    
    # Backward pass
    tau = np.zeros(n_joints)
    f_next = np.zeros(3)
    n_next = np.zeros(3)
    
    joint_idx = n_joints - 1
    for i in range(n_bodies - 1, -1, -1):
        f = f_next + F[i]
        
        p_com_i = fk.p_com_all[i]
        if i < n_bodies - 1 and joint_idx >= 0:
            p_next = fk.p_joint[min(joint_idx + 1, n_joints - 1)] if joint_idx + 1 < n_joints else fk.p_ee
        else:
            p_next = fk.p_ee
        
        r_com = p_com_i - p_next if i < n_bodies - 1 else np.zeros(3)
        n = N[i] + n_next + np.cross(f_next, r_com)
        
        if params.joint_types[i] == 'revolute' and joint_idx >= 0:
            tau[joint_idx] = np.dot(n, fk.k[joint_idx])
            joint_idx -= 1
        
        f_next = f
        n_next = n
    
    # 조립
    c = np.zeros(6 + n_joints)
    c[0:3] = f_next
    c[3:6] = n_next
    c[6:] = tau
    
    return c


# ============================================================
#  모터 다이나믹스
# ============================================================

def motor_dynamics(tau_actual: np.ndarray, tau_cmd: np.ndarray, 
                   params: KariArmParams) -> Tuple[np.ndarray, np.ndarray]:
    """
    모터 다이나믹스 (1차 지연 + Saturation)
    
    Args:
        tau_actual: 현재 실제 토크 [7,]
        tau_cmd: 명령 토크 [7,]
        params: 파라미터
    
    Returns:
        tau_dot: 토크 변화율 [7,]
        tau_sat: saturation 적용된 명령 토크 [7,]
    """
    tau_actual = tau_actual.flatten()
    tau_cmd = tau_cmd.flatten()
    
    # Saturation
    tau_sat = np.clip(tau_cmd, -params.motor_tau_max, params.motor_tau_max)
    
    # 1차 지연
    tau_dot = np.zeros(params.n_joints)
    for i in range(params.n_joints):
        if abs(tau_sat[i]) >= abs(tau_actual[i]):
            tau_m = params.motor_tau_up[i]
        else:
            tau_m = params.motor_tau_down[i]
        tau_dot[i] = (tau_sat[i] - tau_actual[i]) / tau_m
    
    return tau_dot, tau_sat


# ============================================================
#  테스트
# ============================================================

if __name__ == "__main__":
    print("=== Left Arm (arm_id='L') ===")
    params_L = KariArmParams(arm_id='L')
    
    p_base = np.array([0.0, 0.0, 0.0])
    q_base = np.array([1.0, 0.0, 0.0, 0.0])
    theta = np.zeros(7)
    
    fk_L = forward_kinematics(params_L, p_base, q_base, theta)
    print(f"Mount pos: {params_L.r_mount}")
    print(f"EE Position: {fk_L.p_ee}")
    
    print("\n=== Right Arm (arm_id='R') ===")
    params_R = KariArmParams(arm_id='R')
    
    fk_R = forward_kinematics(params_R, p_base, q_base, theta)
    print(f"Mount pos: {params_R.r_mount}")
    print(f"EE Position: {fk_R.p_ee}")
    
    # Mass Matrix
    H_L = compute_mass_matrix(fk_L, params_L)
    H_R = compute_mass_matrix(fk_R, params_R)
    print(f"\nMass Matrix L shape: {H_L.shape}, Total mass: {H_L[0,0]:.2f} kg")
    print(f"Mass Matrix R shape: {H_R.shape}, Total mass: {H_R[0,0]:.2f} kg")
    
    print("\n테스트 완료!")
