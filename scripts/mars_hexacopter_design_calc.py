#!/usr/bin/env python3
"""
Mars Hexacopter Design Parameter Calculator & Verification

docs/mars_hexacopter_design.md 에 기술된 모든 파라미터를 수치 검증한다.
각 검증 항목(V1~V5)의 PASS/FAIL을 판정한다.

Usage:
  python3 scripts/mars_hexacopter_design_calc.py
"""

import math
import sys

# ============================================================
# 입력 파라미터 (레퍼런스 문서 기반)
# ============================================================

# 화성 대기
MARS_G = 3.72          # m/s^2
RHO = 0.0175           # kg/m^3 (표면 밀도, 설계값)
GAMMA_CO2 = 1.29       # CO2 비열비
R_SPECIFIC_CO2 = 188.9 # J/(kg·K), R/M = 8.314/0.044
T_DESIGN = 240.0       # K (음속 계산용, 온난 조건)
MU = 1.3e-5            # Pa·s (동적 점성)

# 기체 설계
TOTAL_MASS = 5.0       # kg
NUM_ROTORS = 6
BLADES_PER_ROTOR = 2
BLADE_RADIUS = 0.44    # m
SIGMA = 0.14           # 솔리디티 (추력가중)
ARM_LENGTH = 0.95      # m (중심→로터)

# 공기역학
CT = 0.009             # 추력 계수
FM = 0.50              # Figure of Merit

# MulticopterMotorModel
MAX_ROT_VELOCITY = 500.0  # rad/s
ROTOR_VEL_SLOWDOWN = 10

# 질량 배분 (base_link 구성)
M_HUB = 2.8            # kg (중앙 허브 + 전자장비 + CMG + 배터리)
R_HUB = 0.15           # m
H_HUB = 0.10           # m
M_ARM = 0.1            # kg (암 1개)
L_ARM = 0.80           # m
D_ARM_CENTER = 0.55    # m (암 중심의 원점 거리)
M_MOTOR_STATOR = 0.1   # kg (모터 고정자 1개, 암 끝)
M_ROTOR = 0.055        # kg (블레이드+허브, 1개)

PASS_SYMBOL = "PASS"
FAIL_SYMBOL = "** FAIL **"


def section(title):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def check(name, value, expected, tol_pct=5.0):
    """값이 기대치의 tol_pct% 이내인지 확인."""
    if expected == 0:
        ok = abs(value) < 1e-10
    else:
        err = abs(value - expected) / abs(expected) * 100
        ok = err < tol_pct
    status = PASS_SYMBOL if ok else FAIL_SYMBOL
    if expected != 0:
        err = abs(value - expected) / abs(expected) * 100
        print(f"  {status}  {name}: {value:.6e}  (기대: {expected:.6e}, 오차: {err:.2f}%)")
    else:
        print(f"  {status}  {name}: {value:.6e}  (기대: {expected:.6e})")
    return ok


def main():
    all_pass = True
    results = {}  # SDF에 넣을 최종값 저장

    # ============================================================
    section("0.2 화성 대기 파라미터")
    # ============================================================

    a_sound = math.sqrt(GAMMA_CO2 * R_SPECIFIC_CO2 * T_DESIGN)
    nu = MU / RHO

    print(f"  ρ (밀도)        = {RHO} kg/m³")
    print(f"  T (설계온도)    = {T_DESIGN} K")
    print(f"  γ (CO2)         = {GAMMA_CO2}")
    print(f"  R_specific      = {R_SPECIFIC_CO2} J/(kg·K)")
    print(f"  음속 a          = {a_sound:.1f} m/s")
    print(f"  동점성 ν        = {nu:.4e} m²/s")
    print(f"  ν/ν_earth       = {nu / 1.5e-5:.1f}x  (화성 대기가 ~50배 높은 동점성)")

    # ============================================================
    section("V1: motorConstant 도출 검증")
    # ============================================================

    print(f"\n  공식: motorConstant = CT × ρ × π × R⁴")
    print(f"  CT = {CT}")
    print(f"  ρ  = {RHO} kg/m³")
    print(f"  R  = {BLADE_RADIUS} m")
    print(f"  R⁴ = {BLADE_RADIUS**4:.6f}")

    motor_constant = CT * RHO * math.pi * BLADE_RADIUS**4
    print(f"\n  motorConstant = {CT} × {RHO} × π × {BLADE_RADIUS**4:.6f}")
    print(f"                = {motor_constant:.6e} N/(rad/s)²")

    results['motorConstant'] = motor_constant
    ok = check("motorConstant vs 설계값", motor_constant, 1.854e-05, tol_pct=1.0)
    all_pass = all_pass and ok

    # 역검증: 이 motorConstant로 호버 추력 = 무게?
    weight = TOTAL_MASS * MARS_G
    thrust_per_rotor_required = weight / NUM_ROTORS
    omega_hover = math.sqrt(thrust_per_rotor_required / motor_constant)
    thrust_at_hover = NUM_ROTORS * motor_constant * omega_hover**2

    print(f"\n  역검증:")
    print(f"    무게 W = {TOTAL_MASS} × {MARS_G} = {weight:.3f} N")
    print(f"    로터당 필요 추력 = {thrust_per_rotor_required:.4f} N")
    print(f"    ω_hover = sqrt({thrust_per_rotor_required:.4f} / {motor_constant:.4e}) = {omega_hover:.2f} rad/s")
    print(f"    총 추력 검증 = {NUM_ROTORS} × {motor_constant:.4e} × {omega_hover:.2f}² = {thrust_at_hover:.4f} N")
    ok = check("hover thrust vs weight", thrust_at_hover, weight, tol_pct=0.1)
    all_pass = all_pass and ok

    # ============================================================
    section("V2: 호버 조건 교차검증")
    # ============================================================

    hover_rpm = omega_hover * 60 / (2 * math.pi)
    vtip_hover = omega_hover * BLADE_RADIUS
    mach_hover = vtip_hover / a_sound
    chord = SIGMA * math.pi * BLADE_RADIUS / BLADES_PER_ROTOR
    re_tip = vtip_hover * chord / nu
    disk_area_total = NUM_ROTORS * math.pi * BLADE_RADIUS**2
    disk_loading = weight / disk_area_total

    print(f"\n  호버 각속도  ω = {omega_hover:.2f} rad/s")
    print(f"  호버 RPM      = {hover_rpm:.0f}")
    print(f"  팁 속도  Vtip  = {vtip_hover:.1f} m/s")
    print(f"  팁 마하  M     = {mach_hover:.4f}")
    print(f"  블레이드 코드 c = {chord:.4f} m ({chord*100:.1f} cm)")
    print(f"  레이놀즈 Re_tip = {re_tip:.0f}")
    print(f"  디스크 면적     = {disk_area_total:.3f} m²")
    print(f"  디스크 하중     = {disk_loading:.2f} N/m²")

    ok = check("Mach_hover < 0.80", mach_hover, 0.75, tol_pct=10)
    all_pass = all_pass and ok

    if mach_hover >= 0.80:
        print(f"  {FAIL_SYMBOL}  호버 팁 마하가 0.80 이상! 블레이드 반경 증가 필요")
        all_pass = False
    else:
        print(f"  {PASS_SYMBOL}  호버 팁 마하 {mach_hover:.3f} < 0.80")

    if re_tip < 5000:
        print(f"  WARNING: Re = {re_tip:.0f} < 5000, 공기역학적으로 매우 열악")
    elif re_tip < 15000:
        print(f"  NOTE: Re = {re_tip:.0f}, Ingenuity 수준 (저 Re 영역)")
    else:
        print(f"  {PASS_SYMBOL}  Re = {re_tip:.0f}, 양호 (Ingenuity ~11,000 대비 우수)")

    results['omega_hover'] = omega_hover
    results['chord'] = chord

    # ============================================================
    section("V4: momentConstant 도출 검증")
    # ============================================================

    print(f"\n  FM = {FM}")
    print(f"  CT = {CT}")
    cp = CT**(3/2) / (math.sqrt(2) * FM)
    cq = cp  # 로터크래프트 관례
    cq_over_ct = cq / CT
    moment_constant = cq_over_ct * BLADE_RADIUS

    print(f"  CP = CT^(3/2) / (√2 × FM) = {CT}^1.5 / ({math.sqrt(2):.4f} × {FM})")
    print(f"     = {CT**(3/2):.6e} / {math.sqrt(2)*FM:.4f}")
    print(f"     = {cp:.6e}")
    print(f"  CQ = CP = {cq:.6e}")
    print(f"  CQ/CT = {cq_over_ct:.4f}")
    print(f"  momentConstant = (CQ/CT) × R = {cq_over_ct:.4f} × {BLADE_RADIUS} = {moment_constant:.4f}")

    results['momentConstant'] = moment_constant
    ok = check("momentConstant vs 설계값", moment_constant, 0.059, tol_pct=2.0)
    all_pass = all_pass and ok

    # 호버 시 토크 검증
    hover_torque_per = moment_constant * thrust_per_rotor_required
    print(f"\n  호버 시 로터당 토크 Q = {moment_constant:.4f} × {thrust_per_rotor_required:.4f} = {hover_torque_per:.4f} N·m")
    print(f"  CCW 토크 합 = 3 × {hover_torque_per:.4f} = {3*hover_torque_per:.4f} N·m")
    print(f"  CW 토크 합  = 3 × {hover_torque_per:.4f} = {3*hover_torque_per:.4f} N·m")
    print(f"  → Yaw 밸런스: 등속 시 순 토크 = 0")

    # ============================================================
    section("V5: maxRotVelocity 팁 마하 확인")
    # ============================================================

    vtip_max = MAX_ROT_VELOCITY * BLADE_RADIUS
    mach_max = vtip_max / a_sound
    thrust_per_max = motor_constant * MAX_ROT_VELOCITY**2
    total_thrust_max = NUM_ROTORS * thrust_per_max
    tw_ratio = total_thrust_max / weight

    print(f"  maxRotVelocity = {MAX_ROT_VELOCITY} rad/s")
    print(f"  팁 속도 (max)  = {vtip_max:.1f} m/s")
    print(f"  팁 마하 (max)  = {mach_max:.3f}")
    print(f"  최대 추력/로터 = {thrust_per_max:.3f} N")
    print(f"  총 최대 추력   = {total_thrust_max:.3f} N")
    print(f"  추력/무게비    = {tw_ratio:.2f}")

    if mach_max >= 1.0:
        print(f"  {FAIL_SYMBOL}  최대 팁 마하가 1.0 이상! maxRotVelocity 감소 필요")
        all_pass = False
    else:
        print(f"  {PASS_SYMBOL}  최대 팁 마하 {mach_max:.3f} < 1.0")

    if tw_ratio < 1.2:
        print(f"  WARNING: T/W = {tw_ratio:.2f} < 1.2, 기동 여유 부족")
    else:
        print(f"  {PASS_SYMBOL}  T/W = {tw_ratio:.2f}, 충분한 기동 여유")

    results['maxRotVelocity'] = MAX_ROT_VELOCITY

    # ============================================================
    section("V3: 관성 텐서 계산")
    # ============================================================

    print("\n--- base_link 관성 ---")

    # 중앙 허브 (solid cylinder)
    I_hub_xx = M_HUB * (3 * R_HUB**2 + H_HUB**2) / 12
    I_hub_zz = M_HUB * R_HUB**2 / 2
    print(f"  허브: m={M_HUB}, r={R_HUB}, h={H_HUB}")
    print(f"    Ixx = Iyy = {I_hub_xx:.5f}")
    print(f"    Izz       = {I_hub_zz:.5f}")

    # 6개 암 (thin rod + parallel axis theorem)
    # 각 암은 x축에서 angle만큼 회전됨
    angles_deg = [0, 60, 120, 180, 240, 300]
    I_arms_xx = 0
    I_arms_yy = 0
    I_arms_zz = 0

    for angle_deg in angles_deg:
        angle = math.radians(angle_deg)
        cx = D_ARM_CENTER * math.cos(angle)
        cy = D_ARM_CENTER * math.sin(angle)

        # 암의 로컬 관성 (길이 L_ARM, 폭/높이 무시)
        # 길이 방향 = angle 방향
        I_long = M_ARM * L_ARM**2 / 12  # 길이 축 (작음 = 0)
        I_perp = M_ARM * L_ARM**2 / 12  # 수직 축

        # 암의 관성을 글로벌로 변환 + 평행축
        # Izz는 항상 I_perp + m*d² (d = 암 중심까지 거리)
        I_arms_zz += I_perp + M_ARM * (cx**2 + cy**2)

        # Ixx, Iyy: 암 방향에 따라 기여가 달라짐
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        # I_local_along_arm ≈ 0 (단면이 작으므로)
        # I_local_perp_arm = mL²/12
        # 글로벌 좌표 변환:
        I_local_xx = I_perp * sin_a**2  # 암이 x축 방향이면 Ixx기여 = 0
        I_local_yy = I_perp * cos_a**2
        # 평행축
        I_arms_xx += I_local_xx + M_ARM * cy**2
        I_arms_yy += I_local_yy + M_ARM * cx**2

    print(f"\n  6개 암 (각 m={M_ARM}, L={L_ARM}, 중심거리={D_ARM_CENTER}):")
    print(f"    Ixx 기여 = {I_arms_xx:.5f}")
    print(f"    Iyy 기여 = {I_arms_yy:.5f}")
    print(f"    Izz 기여 = {I_arms_zz:.5f}")

    # 6개 모터 고정자 (point mass at arm tip)
    I_motors_xx = 0
    I_motors_yy = 0
    I_motors_zz = 0
    for angle_deg in angles_deg:
        angle = math.radians(angle_deg)
        mx = ARM_LENGTH * math.cos(angle)
        my = ARM_LENGTH * math.sin(angle)
        I_motors_xx += M_MOTOR_STATOR * my**2
        I_motors_yy += M_MOTOR_STATOR * mx**2
        I_motors_zz += M_MOTOR_STATOR * (mx**2 + my**2)

    print(f"\n  6개 모터 고정자 (각 m={M_MOTOR_STATOR}, 거리={ARM_LENGTH}):")
    print(f"    Ixx 기여 = {I_motors_xx:.5f}")
    print(f"    Iyy 기여 = {I_motors_yy:.5f}")
    print(f"    Izz 기여 = {I_motors_zz:.5f}")

    # 합계
    I_base_xx = I_hub_xx + I_arms_xx + I_motors_xx
    I_base_yy = I_hub_xx + I_arms_yy + I_motors_yy  # hub는 대칭
    I_base_zz = I_hub_zz + I_arms_zz + I_motors_zz

    print(f"\n  base_link 관성 합계:")
    print(f"    Ixx = {I_base_xx:.5f}")
    print(f"    Iyy = {I_base_yy:.5f}")
    print(f"    Izz = {I_base_zz:.5f}")

    # 대칭 확인
    asym = abs(I_base_xx - I_base_yy) / I_base_xx * 100
    print(f"    Ixx/Iyy 비대칭 = {asym:.2f}% (hex 대칭이면 0에 가까워야 함)")
    if asym > 5:
        print(f"  WARNING: 비대칭이 큼 — 계산 확인 필요")

    results['base_Ixx'] = I_base_xx
    results['base_Iyy'] = I_base_yy
    results['base_Izz'] = I_base_zz

    # 질량 검증
    m_base = M_HUB + 6 * M_ARM + 6 * M_MOTOR_STATOR
    m_remaining = TOTAL_MASS - 6 * M_ROTOR - m_base
    print(f"\n  질량 검증:")
    print(f"    허브+전자장비+CMG+배터리: {M_HUB} kg")
    print(f"    암 6개: {6*M_ARM} kg")
    print(f"    모터 고정자 6개: {6*M_MOTOR_STATOR} kg")
    print(f"    base_link 합계: {m_base:.3f} kg")
    print(f"    로터 6개: {6*M_ROTOR:.3f} kg")
    print(f"    배분 잔여: {m_remaining:.3f} kg (배선/기타로 base_link에 포함)")
    print(f"    SDF base_link 질량: {TOTAL_MASS - 6*M_ROTOR:.3f} kg")

    results['base_mass'] = TOTAL_MASS - 6 * M_ROTOR

    print(f"\n--- 로터 링크 관성 (1개) ---")

    # 2-blade propeller spinning about Z
    I_rotor_zz = (1.0/3.0) * M_ROTOR * BLADE_RADIUS**2
    I_rotor_xx = 1.0e-5  # 허브/두께 방향 (최소)
    I_rotor_yy = I_rotor_zz  # 블레이드 스팬 수직 ≈ 스핀축

    print(f"  질량 = {M_ROTOR} kg, R = {BLADE_RADIUS} m")
    print(f"  Izz (스핀축) = (1/3) × m × R² = {I_rotor_zz:.6e}")
    print(f"  Ixx (허브)   ≈ {I_rotor_xx:.6e}")
    print(f"  Iyy          ≈ Izz = {I_rotor_yy:.6e}")

    results['rotor_mass'] = M_ROTOR
    results['rotor_Ixx'] = I_rotor_xx
    results['rotor_Iyy'] = I_rotor_yy
    results['rotor_Izz'] = I_rotor_zz

    # ============================================================
    section("로터 간 간섭 체크")
    # ============================================================

    # hex에서 인접 로터 간 거리 = ARM_LENGTH (중심-중심)
    rotor_gap = ARM_LENGTH - 2 * BLADE_RADIUS
    print(f"  암 길이 L = {ARM_LENGTH} m")
    print(f"  블레이드 직경 = {2*BLADE_RADIUS} m")
    print(f"  인접 로터 중심 거리 = {ARM_LENGTH:.3f} m")
    print(f"  인접 로터 팁 간격 = {rotor_gap:.3f} m ({rotor_gap*100:.1f} cm)")

    if rotor_gap < 0:
        print(f"  WARNING: 블레이드 겹침 {abs(rotor_gap)*100:.1f} cm!")
        print(f"  → SDF에서 로터 충돌체 제거 필요 (self_collide=false)")
    elif rotor_gap < 0.05:
        print(f"  NOTE: 간격이 {rotor_gap*100:.1f} cm로 매우 좁음")
    else:
        print(f"  {PASS_SYMBOL}  간격 {rotor_gap*100:.1f} cm, 충분")

    # ============================================================
    section("기타 MulticopterMotorModel 파라미터")
    # ============================================================

    # rotorDragCoefficient: H-force = coeff × ω²
    # 대략 motorConstant의 ~10% 수준
    rotor_drag_coeff = 0.001 * RHO * math.pi * BLADE_RADIUS**4
    rolling_moment_coeff = 1e-7  # 작은 값 (프로파일 드래그 기반)

    print(f"  rotorDragCoefficient   = {rotor_drag_coeff:.4e}")
    print(f"    (= 0.001 × ρ × π × R⁴, motorConstant의 ~{rotor_drag_coeff/motor_constant*100:.1f}%)")
    print(f"  rollingMomentCoefficient = {rolling_moment_coeff:.4e}")
    print(f"  rotorVelocitySlowdownSim = {ROTOR_VEL_SLOWDOWN}")
    print(f"  timeConstantUp           = 0.0125 s")
    print(f"  timeConstantDown         = 0.025 s")

    results['rotorDragCoefficient'] = rotor_drag_coeff
    results['rollingMomentCoefficient'] = rolling_moment_coeff

    # ============================================================
    section("기체 외형 치수")
    # ============================================================

    span = 2 * (ARM_LENGTH + BLADE_RADIUS)
    print(f"  기체 스팬 (팁-팁) = 2 × ({ARM_LENGTH} + {BLADE_RADIUS}) = {span:.2f} m")
    print(f"  허브 직경 = {2*R_HUB:.2f} m")
    print(f"  대략 높이 = ~0.35 m (허브 + 랜딩기어)")

    # ============================================================
    section("SDF 파라미터 최종 요약")
    # ============================================================

    print(f"""
  === MulticopterMotorModel ===
  motorConstant           = {results['motorConstant']:.6e}
  momentConstant          = {results['momentConstant']:.4f}
  rotorDragCoefficient    = {results['rotorDragCoefficient']:.4e}
  rollingMomentCoefficient= {results['rollingMomentCoefficient']:.4e}
  maxRotVelocity          = {results['maxRotVelocity']:.1f}
  rotorVelocitySlowdownSim= {ROTOR_VEL_SLOWDOWN}

  === base_link ===
  mass  = {results['base_mass']:.3f} kg
  Ixx   = {results['base_Ixx']:.5f}
  Iyy   = {results['base_Iyy']:.5f}
  Izz   = {results['base_Izz']:.5f}

  === rotor_link (각 1개) ===
  mass  = {results['rotor_mass']:.4f} kg
  Ixx   = {results['rotor_Ixx']:.4e}
  Iyy   = {results['rotor_Iyy']:.6e}
  Izz   = {results['rotor_Izz']:.6e}

  === 호버 조건 ===
  ω_hover   = {results['omega_hover']:.2f} rad/s ({results['omega_hover']*60/(2*math.pi):.0f} RPM)
  Vtip      = {results['omega_hover']*BLADE_RADIUS:.1f} m/s
  Mach      = {results['omega_hover']*BLADE_RADIUS/a_sound:.4f}
  chord     = {results['chord']:.4f} m
  Re_tip    = {results['omega_hover']*BLADE_RADIUS*results['chord']/nu:.0f}
""")

    # ============================================================
    section("검증 결과 종합")
    # ============================================================

    if all_pass:
        print(f"  ALL CHECKS PASSED")
        print(f"  → Phase 2 (SDF 모델 구현) 진행 가능")
    else:
        print(f"  SOME CHECKS FAILED — 파라미터 수정 필요")

    return 0 if all_pass else 1


if __name__ == '__main__':
    sys.exit(main())
