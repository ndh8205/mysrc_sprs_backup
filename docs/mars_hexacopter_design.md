# Mars Hexacopter Design Reference

CMG-augmented Mars rotorcraft simulation을 위한 설계 레퍼런스 문서.
모든 파라미터는 NASA 기술 보고서에서 도출하며, 각 값의 출처와 계산 과정을 명시한다.

---

## 0.1 NASA 레퍼런스

### Ingenuity (Mars Helicopter Technology Demonstrator)
| 파라미터 | 값 | 출처 |
|----------|-----|------|
| 총 질량 | 1.8 kg | JPL Press Kit |
| 로터 구성 | 동축반전 2로터, 각 2블레이드 | NASA/TM-20240001510 |
| 로터 반경 R | 0.605 m | NASA/TM-20240001510 (Koning 2024) |
| 솔리디티 σ (추력가중) | 0.1478 | NASA/TM-20240001510 |
| 에어포일 (외측) | CLF5605 | NASA/TM-20240001510 |
| 호버 RPM | 2400-2700 | Schatzman (2024), NASA/TM-20240000639 |
| 팁 속도 (2537 RPM) | ~161 m/s | 계산: 265.6 rad/s × 0.605m |
| 팁 마하 | 0.63-0.76 | Dull & Cuyler (2022) |
| Figure of Merit | ~0.48 | JPL 25-ft Space Simulator 테스트 |
| Re (팁 기준) | ~11,000 | Comparing 3D and 2D CFD (NASA) |
| 동축 로터 간격 | ~10 cm (8-9% D) | Ingenuity Rotor Geometry TM |

### Mars Science Helicopter (MSH)
| 파라미터 | 값 | 출처 |
|----------|-----|------|
| 총 질량 | 31 kg | NASA/TM-2020-220485 (Johnson 2020) |
| 구성 | **헥사콥터** (6로터) | MSH Summary AIAA ASCEND |
| 블레이드/로터 | 4 | MSH Summary |
| 로터 반경 R | 0.64 m | MSH Summary |
| 솔리디티 σ | 0.142 | MSH Rotor Geometry TM |
| 설계 RPM | 2782 | MSH Summary |
| 팁 마하 | 0.80 | MSH Summary |
| 페이로드 | 2-5 kg | MSH Summary |
| 기체 직경 | ~4.0 m | MSH Summary |

### Advanced Mars Helicopter (AMH)
| 파라미터 | 값 | 출처 |
|----------|-----|------|
| 총 질량 | ~5 kg | Withrow-Maser & Johnson (NASA) |
| 구성 | 동축반전 | AMH Design Paper |
| 페이로드 | ~1.3 kg | AMH Design Paper |

### 레퍼런스 문서 URL
1. [Ingenuity Rotor Geometry (Koning 2024)](https://rotorcraft.arc.nasa.gov/Publications/files/1699_Koning_TM_020224.pdf)
2. [MSH Conceptual Design (Johnson 2020)](https://rotorcraft.arc.nasa.gov/Publications/files/MSH_WJohnson_TM2020rev.pdf)
3. [MSH Summary AIAA ASCEND](https://rotorcraft.arc.nasa.gov/Publications/files/MSH_summary_AIAA_ASCEND_final.pdf)
4. [MSH Rotor Geometry (Koning 2025)](https://ntrs.nasa.gov/api/citations/20240013701/downloads/msh-airfoil-rotor-geom-20250203.pdf)
5. [Ingenuity Performance Data (Schatzman 2024)](https://ntrs.nasa.gov/api/citations/20240000639/downloads/1684_Schatzman_Final_011724.pdf)
6. [AMH Design (Withrow-Maser)](https://rotorcraft.arc.nasa.gov/Publications/files/An_Advanced_Helicopter_Design_SW_ASCEND_final.pdf)
7. [Mars Atmosphere Model (NASA GRC)](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/mars-atmosphere-equation-metric/)
8. [Speed of Sound on Mars](https://aerospaceweb.org/question/atmosphere/q0249.shtml)
9. [Mars Hexacopter Dynamics (Singh, SJSU)](https://www.sjsu.edu/ae/docs/project-thesis/Raghuvir.Singh-S23.pdf)

---

## 0.2 화성 대기 파라미터

### 표면 조건 (Jezero Crater 기준, MOLA 고도 ~-4500m)

| 파라미터 | 기호 | 값 | 출처 | 비고 |
|----------|------|-----|------|------|
| 표면 밀도 | ρ | **0.0175 kg/m³** | NASA GRC 모델 | 계절/온도 변화: 0.015-0.020 |
| 표면 압력 | p | 636 Pa (~0.636 kPa) | NASA GRC 모델 | 지구의 ~0.6% |
| 표면 온도 | T | 210 K (-63°C) | 설계 참조온도 | 일변화: 145-245 K |
| 중력가속도 | g | 3.72 m/s² | — | |
| 대기 조성 | — | 95.3% CO₂ | — | |

### 음속
```
a = sqrt(γ × R_specific × T)
  = sqrt(1.29 × 188.9 × 210)
  = sqrt(51,191)
  = 226 m/s  (at 210K)

a = sqrt(1.29 × 188.9 × 240)
  = sqrt(58,504)
  = 242 m/s  (at 240K)
```
- γ(CO₂) = 1.29
- R_specific(CO₂) = R/M = 8.314/0.044 = 188.9 J/(kg·K)
- **설계값: a = 240 m/s** (보수적, 온난 조건 기준)

### 점성
| 파라미터 | 기호 | 값 | 출처 |
|----------|------|-----|------|
| 동적 점성 | μ | 1.3 × 10⁻⁵ Pa·s | 문헌값 |
| 동점성 | ν = μ/ρ | 7.43 × 10⁻⁴ m²/s | 계산 (ρ=0.0175) |

참고: 지구 ν ≈ 1.5 × 10⁻⁵ → 화성은 **~50배 높음** → Re 크게 감소

---

## 0.3 로터 공기역학

### 추력 계수 (CT)
로터크래프트 관례 (helicopter convention):
```
T = CT × ρ × A × (ωR)²
```
여기서 A = πR² (디스크 면적), ωR = Vtip

**CT 값 선정:**
- Ingenuity 호버 역산: CT ≈ 0.007-0.012 (집합피치 의존)
- MSH 설계: CT/σ ≈ 0.05-0.08
- **설계값: CT = 0.009** (중간값, 고정피치 로터 기준)

선정 근거: Ingenuity의 CT가 0.007-0.012 범위이고, 고정피치 2블레이드 로터의
일반적 작동점. 가변피치가 없으므로 낮은 쪽 선택.

### Figure of Merit (FM)
```
FM = C_T^(3/2) / (√2 × C_P)
```
- Ingenuity 실측: FM ≈ 0.48 (JPL 테스트)
- MSH 예측: FM ≈ 0.5-0.7
- **설계값: FM = 0.50** (Ingenuity 실측 기반, 보수적)

### 토크 계수 (CQ)
CQ = CP (로터크래프트 관례에서)
```
CP = CT^(3/2) / (√2 × FM)
   = 0.009^(3/2) / (√2 × 0.50)
   = 8.544e-4 / 0.7071
   = 1.208e-3
```
**CQ = 1.208e-3**

### CQ/CT 비
```
CQ/CT = 1.208e-3 / 0.009 = 0.1342
```
이 비율은 화성의 높은 프로파일 드래그 (저 Re) 를 반영.
지구 프로펠러 CQ/CT ≈ 0.05-0.10 대비 높은 값.

---

## 0.4 motorConstant 도출

### Gazebo MulticopterMotorModel 추력 모델
```
F = motorConstant × ω²
```

### 로터크래프트 추력 공식에서 도출
```
F = CT × ρ × πR² × (ωR)²
  = CT × ρ × πR² × ω²R²
  = CT × ρ × π × R⁴ × ω²
```

따라서:
```
motorConstant = CT × ρ × π × R⁴
```

### 수치 대입
```
motorConstant = 0.009 × 0.0175 × π × 0.44⁴
              = 0.009 × 0.0175 × 3.14159 × 0.03748
              = 0.009 × 0.0175 × 0.11773
              = 1.854 × 10⁻⁵  [N/(rad/s)²]
```

### 역검증 (호버 조건)
```
W = m × g = 5.0 × 3.72 = 18.6 N
T_per_rotor = 18.6 / 6 = 3.1 N
ω_hover = sqrt(T / motorConstant) = sqrt(3.1 / 1.854e-5) = sqrt(167,204) = 409 rad/s
```

---

## 0.5 momentConstant 도출

### Gazebo MulticopterMotorModel 토크 모델
```
Q = momentConstant × F
  = momentConstant × motorConstant × ω²
```

### 로터크래프트 토크 공식
```
Q = CQ × ρ × πR² × (ωR)² × R
  = CQ × ρ × π × R⁵ × ω²
```

### momentConstant 관계
```
momentConstant = Q/F = (CQ × ρ × π × R⁵ × ω²) / (CT × ρ × π × R⁴ × ω²)
               = (CQ/CT) × R
               = 0.1342 × 0.44
               = 0.059
```

**momentConstant = 0.059**

### 의미
- 1 N 추력 당 0.059 N·m 반작용 토크
- 호버 시 로터당: Q = 0.059 × 3.1 = 0.183 N·m
- CCW 로터 3개 토크 = CW 로터 3개 토크 → yaw 밸런스

---

## 0.6 기체 질량/관성 예산

### 질량 배분

| 항목 | 질량 (kg) | 비율 | 비고 |
|------|----------|------|------|
| 구조/프레임 | 1.5 | 30% | 탄소섬유 튜브+허브 |
| 모터 고정자 (6) | 0.6 | 12% | ~100g/모터 |
| 배터리 | 0.8 | 16% | 고에너지밀도 |
| 아비오닉스 | 0.3 | 6% | FC + 센서 |
| CMG 클러스터 | 0.5 | 10% | 4× nano-CMG |
| 페이로드 마진 | 0.3 | 6% | |
| 랜딩기어 | 0.2 | 4% | |
| 배선/기타 | 0.47 | 9.4% | |
| **base_link 소계** | **4.67** | **93.4%** | |
| 로터/블레이드 (6) | 0.33 | 6.6% | 55g/로터 (블레이드+허브) |
| **총 질량** | **5.00** | **100%** | |

### 관성 텐서 — base_link

모델: 중앙 허브(실린더) + 6개 암(막대) + 6개 모터 고정자(점질량)

**중앙 허브** (m_hub ≈ 2.8 kg, r = 0.15 m, h = 0.10 m):
```
I_xx = I_yy = m(3r² + h²)/12 = 2.8(3×0.0225 + 0.01)/12 = 2.8 × 0.00646 = 0.0181
I_zz = mr²/2 = 2.8 × 0.0225/2 = 0.0315
```

**6개 암** (각 m_arm = 0.1 kg, 길이 0.80 m, 중심이 원점에서 0.55m):
각 암은 막대(I = mL²/12)에 평행축 정리 적용.
6개가 60° 간격이므로 대칭 → Ixx = Iyy.

```
I_arm_local = m × L²/12 = 0.1 × 0.64/12 = 0.00533  (암 자체 관성)
I_arm_parallel = m × d² = 0.1 × 0.55² = 0.03025      (평행축)
I_arm_total_per = 0.00533 + 0.03025 = 0.03558

Ixx 기여 (6개 합, hex 대칭):
  = 6 × I_arm_total × (sin²θ + cos²θ 가중평균)
  → 대칭이므로 Ixx = Iyy = 3 × I_arm_total = 0.107
  → Izz = 6 × I_arm_total = 0.214
```

**6개 모터 고정자** (각 0.1 kg, 거리 0.95 m):
```
Ixx = Iyy = 3 × m × L² = 3 × 0.1 × 0.9025 = 0.271
Izz = 6 × m × L² = 6 × 0.1 × 0.9025 = 0.542
```

**합계:**
```
Ixx = Iyy = 0.018 + 0.107 + 0.271 = 0.396 ≈ 0.40
Izz = 0.032 + 0.214 + 0.542 = 0.788 ≈ 0.79
```

**검증 스크립트(Phase 1)에서 정밀 계산 후 확정**

### 관성 텐서 — 로터 링크 (각 1개)

모델: 2블레이드 프로펠러 (질량 0.055 kg, 반경 0.44 m)
두 블레이드가 직선으로 배치, 조인트 축 = Z

```
I_spin (Izz) = 2 × (1/3) × (m/2) × R²
             = (1/3) × m × R²
             = (1/3) × 0.055 × 0.1936
             = 0.00355

I_flap (Ixx 또는 Iyy, 블레이드 면 수직):
  → 블레이드가 매우 얇으므로 ≈ 0
  → 실질적으로 I_flap ≈ m × t²/12 (t는 두께 ~5mm)
  → ≈ 0.055 × 0.000025/12 ≈ 1.1e-07 (무시 가능)

I_xx (블레이드 스팬과 수직, 수평면 내):
  → 블레이드가 한 방향으로 배치: Ixx ≈ 0 (스팬 축)
  → Iyy ≈ Izz ≈ 0.00355 (perpendicular axis theorem)
```

x500_base 패턴과 비교하여 SDF 값 결정:
```
Ixx = 1.0e-05   (허브/두께 방향, 최소)
Iyy = 3.55e-03  (블레이드 스팬 수직)
Izz = 3.55e-03  (스핀 축)
```

**Note**: Iyy ≈ Izz >> Ixx 패턴은 x500_base와 동일.
검증 스크립트에서 정밀 확인.

---

## 0.7 호버 조건 검증

| 항목 | 값 | 계산식 |
|------|-----|--------|
| 화성 무게 W | 18.6 N | 5.0 × 3.72 |
| 로터당 추력 | 3.1 N | 18.6 / 6 |
| 호버 ω | 409 rad/s | sqrt(3.1 / 1.854e-5) |
| 호버 RPM | 3907 | 409 × 60/(2π) |
| 팁 속도 | 180 m/s | 409 × 0.44 |
| **팁 마하** | **0.750** | 180 / 240 |
| Re (팁) | ~23,600 | Vtip × c / ν = 180 × 0.097 / 7.43e-4 |
| 디스크 하중 | 5.10 N/m² | 18.6 / (6 × π × 0.44²) |
| 추력/무게비 (max) | 1.44 | (500/409)² |
| 최대 팁 마하 | 0.917 | 500 × 0.44 / 240 |

### Ingenuity 대비
| 항목 | Ingenuity | 본 설계 | 비고 |
|------|-----------|---------|------|
| 팁 마하 (호버) | 0.63-0.72 | 0.75 | 유사 범위 |
| Re (팁) | ~11,000 | ~23,600 | 2배 이상 → 공기역학적 유리 |
| 블레이드 코드 | ~14 cm | ~10 cm | 유사 |
| 디스크 하중 | 2.91 N/m² | 5.10 N/m² | 높지만 합리적 |

---

## 0.8 가정 및 한계점

### 가정
1. **F = kω² 모델**: 블레이드 요소 이론(BET) 대신 단순화된 추력 모델 사용
2. **고정 CT**: 피치 변화 없음 (고정피치 로터), CT는 ω에 무관하다고 가정
3. **지면 효과 무시**: MulticopterMotorModel에 포함되지 않음
4. **동체 항력 무시**: 고속 전진비행 시 부정확
5. **로터 간 간섭 무시**: 인접 로터 후류(downwash) 상호작용 미모델링
6. **고도별 밀도 변화 무시**: motorConstant 고정 (실제로는 고도 ↑ → ρ ↓ → 추력 ↓)
7. **FM=0.50 가정**: Ingenuity 실측 기반이나, 다른 블레이드 설계에서는 다를 수 있음
8. **점성/Re 효과 미반영**: CT, CQ는 Re에 의존하나, 평균값 사용

### 한계
- **transonic 영역 미반영**: 팁 마하 0.75에서 압축성 효과 시작될 수 있으나,
  MulticopterMotorModel은 비압축 모델
- **에어포일 특성 미반영**: CLF5605 등 특수 에어포일의 Cl/Cd 특성을
  직접 반영하지 않고 통합 CT/CQ 값으로 대체
- **구조 유연성 무시**: 블레이드 플래핑, 리드-래그 동역학 미포함

### 이 설계가 유효한 범위
- 호버 및 저속 비행 (전진비 < 0.1)
- 자세 제어 알고리즘 (PID, CMG) 검증
- 추력-무게비 1.0-1.4 범위 작동
- **CMG 보강 효과 비교 연구에는 충분한 정밀도**

---

## 0.9 시뮬레이션 검증 결과

### Phase 1 검증 (사전 계산, `scripts/mars_hexacopter_design_calc.py`)

| # | 항목 | 결과 | 상세 |
|---|------|------|------|
| V1 | motorConstant | PASS | 1.854561e-05 (오차 0.03%) |
| V2 | 호버 조건 | PASS | Mach=0.744, Re=23,432 |
| V3 | 관성 텐서 | PASS | Ixx=Iyy=0.396, Izz=0.787, 비대칭 0.00% |
| V4 | momentConstant | PASS | 0.0590 (오차 0.05%) |
| V5 | 최대 팁 마하 | PASS | Mach=0.910 < 1.0, T/W=1.50 |

### Phase 3 검증 (Gazebo 시뮬레이션, `scripts/mars_hexacopter_dynamics_test.py`)

| # | 항목 | 결과 | 측정값 | 기대값 |
|---|------|------|--------|--------|
| V6 | 자유낙하 | PASS | g = 3.709 m/s² | 3.72 (오차 0.3%) |
| V7 | 호버링 (409 rad/s) | PASS | avg_v = -0.071 m/s, Z range = 0.57m | dZ ≈ 0 |
| V8 | 추력 비례 (450 rad/s) | PASS | k = 1.8545e-05 | 1.8546e-05 (오차 0.0%) |
| V9 | Yaw 밸런스 | PASS | yaw rate std = 0.2°/s | 각가속도 ≈ 0 |

### SDF 최종 확정값

```
motorConstant            = 1.854561e-05
momentConstant           = 0.0590
rotorDragCoefficient     = 2.0606e-06
rollingMomentCoefficient = 1e-07
maxRotVelocity           = 500.0
rotorVelocitySlowdownSim = 10

base_link: mass=4.670, Ixx=Iyy=0.396, Izz=0.787
rotor:     mass=0.055, Ixx=1e-5, Iyy=Izz=3.55e-3
```

---

## 변경 이력
| 날짜 | 내용 |
|------|------|
| 2026-03-03 | 초안 작성 |
| 2026-03-03 | Phase 1~3 검증 완료, 시뮬레이션 결과 추가 |
