# ========================================================================
# 최종 수정일: 2025-08-05
# GCO 시뮬레이션 및 시각화 Python 스크립트
#
# 기능:
# 1. J2 섭동을 고려한 GCO(General Circular Orbit) 상대 운동 시뮬레이션
# 2. LVLH 좌표계와 ECI 좌표계 간의 변환
# 3. 2D 및 3D 궤도 시각화 (모든 축 비율 보정 포함)
# ========================================================================

# 필요한 라이브러리 설치 (최초 1회 실행)
# pip install numpy matplotlib

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# -- 3D 플롯의 축 비율을 동일하게 맞추는 헬퍼 함수 --
def set_axes_equal(ax):
    """
    3D 플롯의 축 비율을 데이터에 맞게 동일하게 설정합니다. (MATLAB의 axis equal)
    x, y, z 축의 데이터 범위를 계산하여 가장 큰 범위를 기준으로
    모든 축의 범위를 동일하게 맞춥니다.
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)
    
    # 가장 큰 범위를 찾습니다.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


# ========================================================================
# %% 시뮬레이션 함수
# ========================================================================

def initialize_orbit_params():
    """궤도 파라미터 초기화"""
    params = {}
    params['deg'] = np.pi / 180
    params['rad'] = 180 / np.pi
    params['mu'] = 398600.441799999971  # km^3/s^2
    params['J2'] = 0.0010827
    params['R_e'] = 6378.137  # km
    params['w_e'] = 7.29e-5  # rad/s
    
    # 기준 궤도 (Reference orbit)
    params['e_ref'] = 0
    params['a_ref'] = 6892.137  # km
    params['h_ref'] = np.sqrt(params['a_ref'] * (1 - params['e_ref']**2) * params['mu'])
    params['i_ref'] = 97.45 * params['deg']
    params['RAAN_ref'] = 90 * params['deg']
    params['omega_ref'] = 90 * params['deg']
    params['theta_ref'] = 0 * params['deg']
    
    # 평균 운동 및 주기
    params['n'] = np.sqrt(params['mu'] / params['a_ref']**3)
    params['T'] = 2 * np.pi / params['n']
    r_ref = params['a_ref']
    i_ref = params['i_ref']
    
    params['s'] = (3 * params['J2'] * params['R_e']**2) / (8 * r_ref**2) * (1 + 3 * np.cos(2 * i_ref))
    params['c'] = np.sqrt(1 + params['s'])
    return params

def initialize_GCO_satellites(phase_angles_deg, radius_km, params):
    """GCO 위성 초기화"""
    satellites = []
    
    # Chief 위성
    satellites.append({
        'initial_state_lvlh': np.zeros((6, 1)),
        'phase_angle': np.nan,
        'color': [0, 0.4470, 0.7410],
        'name': 'Chief Sat'
    })
    
    # Deputy 위성
    phase_rad = phase_angles_deg[0] * params['deg']
    r_0 = radius_km / 2 * np.sin(phase_rad)
    v_0 = radius_km * params['n'] / 2 * np.cos(phase_rad)
    x_0, x_0_dot = r_0, v_0
    y_0, y_0_dot = 2 * x_0_dot / params['n'], -2 * params['n'] * x_0
    z_0, z_0_dot = np.sqrt(3) * x_0, np.sqrt(3) * x_0_dot
    
    satellites.append({
        'initial_state_lvlh': np.array([x_0, x_0_dot, y_0, y_0_dot, z_0, z_0_dot]).reshape(6, 1),
        'phase_angle': phase_angles_deg[0],
        'color': [0.8500, 0.3250, 0.0980],
        'name': f'Deputy Sat ({phase_angles_deg[0]:.0f}°)'
    })
    
    return satellites

def run_GCO_simulation(phase_angles_deg, cluster_radius_km, sim_time_orbits):
    """GCO 시뮬레이션 실행"""
    params = initialize_orbit_params()
    satellites = initialize_GCO_satellites(phase_angles_deg, cluster_radius_km, params)
    num_sats = len(satellites)
    
    sim_time = sim_time_orbits * params['T']
    dt = 10
    time = np.arange(0, sim_time + dt, dt)
    num_steps = len(time)
    
    satellites_lvlh = np.zeros((6, num_sats, num_steps))
    
    print('수치 적분을 사용한 LVLH 시뮬레이션 진행 중 (계수 수정 모델 적용)...')
    for sat_idx in range(num_sats):
        satellites_lvlh[:, sat_idx, :] = \
            compute_numerical_GCO(satellites[sat_idx]['initial_state_lvlh'], time, params)
    
    print('ECI 상태 계산 및 좌표 변환 진행 중...')
    satellites_eci = np.zeros((6, num_sats, num_steps))
    
    chief_oe = np.array([params['a_ref'], params['e_ref'], params['i_ref'], params['RAAN_ref'], params['omega_ref'], params['theta_ref']])
    chief_eci_states = np.zeros((6, num_steps))
    for k in range(num_steps):
        current_theta = params['theta_ref'] + params['n'] * time[k]
        chief_oe[5] = current_theta
        r, v = oe2eci(chief_oe, params['mu'])
        chief_eci_states[:, k] = np.vstack((r, v)).flatten()
        
    satellites_eci[:, 0, :] = chief_eci_states
    
    for k in range(num_steps):
        r_chief_eci, v_chief_eci = chief_eci_states[0:3, k], chief_eci_states[3:6, k]
        rho, rho_dot = satellites_lvlh[[0, 2, 4], 1, k], satellites_lvlh[[1, 3, 5], 1, k]
        r_dep_eci, v_dep_eci = lvlh2eci(r_chief_eci, v_chief_eci, rho, rho_dot)
        satellites_eci[:, 1, k] = np.hstack((r_dep_eci, v_dep_eci))
        
    params['satellites'] = satellites
    return time, satellites_lvlh, satellites_eci, params

def compute_numerical_GCO(initial_state, time, params):
    """수치 적분을 통해 GCO 궤도 계산"""
    num_steps = len(time)
    state_history = np.zeros((6, num_steps))
    X = initial_state.flatten()
    state_history[:, 0] = X
    u = np.zeros(3)
    dt = time[1] - time[0]
    for i in range(1, num_steps):
        X = rk4(nonline_orbit_cw_J2, X, u, params, dt)
        state_history[:, i] = X
    return state_history

def nonline_orbit_cw_J2(X, u, params):
    """비선형 J2 효과 포함된 동역학 모델"""
    x, vx, y, vy, z, vz = X
    n, c = params['n'], params['c']
    x_2dot = (5*c**2 - 2)*n**2*x + 2*n*c*vy + u[0]
    y_2dot = -2*n*c*vx + u[1]
    z_2dot = -(3*c**2 - 2)*n**2*z + u[2]
    return np.array([vx, x_2dot, vy, y_2dot, vz, z_2dot])

def rk4(Func, x, u, params, delt):
    """4차 룽게-쿠타 적분기"""
    k1 = Func(x, u, params) * delt
    k2 = Func(x + k1 * 0.5, u, params) * delt
    k3 = Func(x + k2 * 0.5, u, params) * delt
    k4 = Func(x + k3, u, params) * delt
    return x + (k1 + 2*(k2 + k3) + k4) / 6.0

# ========================================================================
# %% 좌표 변환 함수
# ========================================================================

def oe2eci(oe, mu):
    """궤도 요소를 ECI 좌표로 변환"""
    a, e, i, RAAN, omega, theta = oe
    p = a * (1 - e**2)
    r_pqw = np.array([p*np.cos(theta)/(1+e*np.cos(theta)), p*np.sin(theta)/(1+e*np.cos(theta)), 0]).reshape(3, 1)
    v_pqw = np.array([-np.sqrt(mu/p)*np.sin(theta), np.sqrt(mu/p)*(e+np.cos(theta)), 0]).reshape(3, 1)
    R3_W = np.array([[np.cos(RAAN),-np.sin(RAAN),0],[np.sin(RAAN),np.cos(RAAN),0],[0,0,1]])
    R1_i = np.array([[1,0,0],[0,np.cos(i),-np.sin(i)],[0,np.sin(i),np.cos(i)]])
    R3_w = np.array([[np.cos(omega),-np.sin(omega),0],[np.sin(omega),np.cos(omega),0],[0,0,1]])
    C_pqw2eci = (R3_W @ R1_i @ R3_w).T
    r_eci = C_pqw2eci @ r_pqw
    v_eci = C_pqw2eci @ v_pqw
    return r_eci, v_eci

def lvlh2eci(r_chief_eci, v_chief_eci, rho, rho_dot):
    """LVLH 좌표를 ECI 좌표로 변환"""
    r_hat = r_chief_eci / np.linalg.norm(r_chief_eci)
    h_vec = np.cross(r_chief_eci, v_chief_eci)
    h_hat = h_vec / np.linalg.norm(h_vec)
    theta_hat = np.cross(h_hat, r_hat)
    C_lvlh2eci = np.vstack((r_hat, theta_hat, h_hat)).T
    
    rho, rho_dot = rho.reshape(3,1), rho_dot.reshape(3,1)
    r_chief_eci, v_chief_eci = r_chief_eci.reshape(3,1), v_chief_eci.reshape(3,1)

    r_dep_eci = r_chief_eci + C_lvlh2eci @ rho
    w = h_vec / (np.linalg.norm(r_chief_eci)**2)
    v_dep_eci = v_chief_eci + C_lvlh2eci @ rho_dot + np.cross(w, (C_lvlh2eci @ rho).flatten()).reshape(3,1)
    return r_dep_eci.flatten(), v_dep_eci.flatten()

# ========================================================================
# %% 플롯 함수
# ========================================================================

def plot_eci_and_lvlh_orbits(satellites_eci, satellites_lvlh, params):
    """ECI 및 LVLH 궤도 통합 시각화"""
    fig = plt.figure(figsize=(13, 6.5))
    fig.suptitle('ECI and LVLH Frame Orbits', fontsize=16)

    r_chief_eci = satellites_eci[0:3, 0, :]
    r_dep_eci = satellites_eci[0:3, 1, :]
    x_lvlh, y_lvlh, z_lvlh = satellites_lvlh[0, 1, :], satellites_lvlh[2, 1, :], satellites_lvlh[4, 1, :]
    chief_color, deputy_color = params['satellites'][0]['color'], params['satellites'][1]['color']
    
    # ECI Frame Plot
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
    x_e, y_e, z_e = params['R_e']*np.cos(u)*np.sin(v), params['R_e']*np.sin(u)*np.sin(v), params['R_e']*np.cos(v)
    ax1.plot_surface(x_e, y_e, z_e, color='blue', alpha=0.4, rstride=1, cstride=1)
    
    ax1.plot(r_chief_eci[0,:], r_chief_eci[1,:], r_chief_eci[2,:], color=chief_color, lw=2, label='Chief Orbit')
    ax1.plot(r_dep_eci[0,:], r_dep_eci[1,:], r_dep_eci[2,:], color=deputy_color, lw=1.5, label='Deputy Orbit')
    ax1.plot(r_chief_eci[0,-1:], r_chief_eci[1,-1:], r_chief_eci[2,-1:], 'o', mfc=chief_color, mec='k', ms=10)
    ax1.plot(r_dep_eci[0,-1:], r_dep_eci[1,-1:], r_dep_eci[2,-1:], 'o', mfc=deputy_color, mec='k', ms=8)

    ax1.set_xlabel('X_{ECI} [km]'), ax1.set_ylabel('Y_{ECI} [km]'), ax1.set_zlabel('Z_{ECI} [km]')
    ax1.set_title('Absolute Orbits in ECI Frame (1 Day)'), ax1.legend(loc='upper right'), ax1.grid(True)
    set_axes_equal(ax1)
    ax1.view_init(elev=25, azim=45)

    # LVLH Frame Plot
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    ax2.plot(x_lvlh, y_lvlh, z_lvlh, color=deputy_color, lw=2, label=params['satellites'][1]['name'])
    ax2.plot([0], [0], [0], 'o', mfc=chief_color, mec='k', ms=10, label='Chief (origin)')
    
    ax2.set_xlabel('X (Radial) [km]'), ax2.set_ylabel('Y (Along-track) [km]'), ax2.set_zlabel('Z (Cross-track) [km]')
    ax2.set_title('Relative Motion in LVLH Frame (1 Day)'), ax2.legend(), ax2.grid(True)
    set_axes_equal(ax2)
    ax2.view_init(elev=25, azim=45)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

def plot_GCO_projections(satellites_state, params):
    """GCO 3D 궤적 및 투영도 시각화"""
    fig = plt.figure(figsize=(10, 8))
    fig.suptitle('GCO 3D Trajectory and Projections', fontsize=16)

    x, y, z = satellites_state[0, 1, :]*1000, satellites_state[2, 1, :]*1000, satellites_state[4, 1, :]*1000
    deputy_color, chief_color = params['satellites'][1]['color'], params['satellites'][0]['color']

    # 3D Relative Trajectory
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(x, y, z, color=deputy_color, lw=2), ax1.plot([0], [0], [0], 'o', mfc=chief_color, mec=chief_color, ms=8)
    ax1.set_xlabel('X (Radial) [m]'), ax1.set_ylabel('Y (Along-track) [m]'), ax1.set_zlabel('Z (Cross-track) [m]')
    ax1.set_title('3D Relative Trajectory'), ax1.grid(True), set_axes_equal(ax1), ax1.view_init(elev=25, azim=45)

    # XY, YZ, XZ Projections
    for i, (ax, data1, data2, labels) in enumerate(zip(
        [fig.add_subplot(222), fig.add_subplot(223), fig.add_subplot(224)],
        [x, y, x], [y, z, z],
        [['X (Radial) [m]', 'Y (Along-track) [m]', 'XY Projection'],
         ['Y (Along-track) [m]', 'Z (Cross-track) [m]', 'YZ Projection'],
         ['X (Radial) [m]', 'Z (Cross-track) [m]', 'XZ Projection']]
    )):
        ax.plot(data1, data2, color=deputy_color, lw=2)
        ax.plot(0, 0, 'o', mfc=chief_color, mec=chief_color, ms=8)
        ax.set_xlabel(labels[0]), ax.set_ylabel(labels[1]), ax.set_title(labels[2])
        ax.grid(True), ax.set_aspect('equal', adjustable='box')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

# ======================================================================
# %% 메인 실행 블록
# ======================================================================
if __name__ == '__main__':
    plt.close('all')
    params = initialize_orbit_params()
    
    # %% 시뮬레이션 파라미터 설정
    phase_angles_deg = [0]
    cluster_radius_km = 0.12
    sim_duration_seconds = 86400
    sim_time_orbits = sim_duration_seconds / params['T']
    
    print(f"궤도 주기: {params['T']/60:.2f} 분")
    print(f"시뮬레이션 시간: 24시간 ({sim_time_orbits:.2f} 궤도)\n")

    # %% Case: J2 포함 시뮬레이션 실행
    time, satellites_lvlh_J2, satellites_eci_J2, params_out = run_GCO_simulation(
        phase_angles_deg, cluster_radius_km, sim_time_orbits)

    # %% 시각화 함수 호출
    plot_eci_and_lvlh_orbits(satellites_eci_J2, satellites_lvlh_J2, params_out)
    plot_GCO_projections(satellites_lvlh_J2, params_out)

    # 모든 플롯 창을 화면에 표시
    plt.show()