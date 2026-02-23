# GAIMSat-1 별센서 시뮬레이션 — 수학 공식 및 물리 모델 정리

> **GAIMSat-1** 6U CubeSat (530 km SSO, 2027 발사) 별센서(Star Tracker) 시뮬레이션의
> 수학 공식, 물리 모델, 다이나믹스를 빠짐없이 정리한 문서.

---

## 목차

1. [개요 및 시뮬레이션 파이프라인](#1-개요-및-시뮬레이션-파이프라인)
2. [FOV(시야각) 계산](#2-fov시야각-계산)
3. [좌표 변환](#3-좌표-변환)
4. [항성 측광: Pogson 공식](#4-항성-측광-pogson-공식)
5. [Gaussian PSF (점확산함수)](#5-gaussian-psf-점확산함수)
6. [모션블러 & 롤링셔터](#6-모션블러--롤링셔터)
7. [노이즈 모델 (7종)](#7-노이즈-모델-7종)
8. [Bayer 패턴 (RGGB)](#8-bayer-패턴-rggb)
9. [Bayer→Gray 변환 5종](#9-bayergray-변환-5종)
10. [별 검출 알고리즘](#10-별-검출-알고리즘)
11. [SNR 지표](#11-snr-지표)
12. [Centroid 정확도 평가](#12-centroid-정확도-평가)
13. [부록: 센서/렌즈 데이터베이스](#13-부록-센서렌즈-데이터베이스)

---

## 1. 개요 및 시뮬레이션 파이프라인

### 1.1 프로젝트 목적

비협조 표적 위성 근접 관측 미션에서 Deputy 위성에 탑재되는 별센서의 성능을 사전 검증하기 위한 end-to-end 시뮬레이션. 천구 좌표 입력부터 센서 물리, 노이즈, Bayer 패턴, 별 검출 및 centroid 추정까지 전 과정을 모사한다.

### 1.2 디렉토리 구조

```
startracker_test/
├── gui_star_simulator.m              ← GUI (실시간 파라미터 조절)
├── main_bayer_simulation.m           ← 메인 스크립트
├── test_all_sensor_combinations.m    ← 18센서×8렌즈 일괄 테스트
├── core/
│   ├── simulate_star_image_realistic.m   ← 핵심 시뮬레이션 엔진
│   ├── get_sensor_preset.m               ← 18종 센서 + 8종 렌즈 DB
│   ├── detect_stars_simple.m             ← 별 검출 (CCL)
│   ├── evaluate_centroid_accuracy.m      ← Centroid 정확도 평가
│   ├── calculate_peak_snr.m              ← Peak SNR
│   ├── calculate_snr.m                   ← Reference SNR
│   ├── bayer_to_gray_direct.m            ← 5종 Bayer→Gray 변환
│   ├── bayer_to_rgb_cfa.m                ← FPGA CFA 디모자이킹
│   └── rgb_to_gray_fpga.m               ← FPGA RGB→Gray
└── data/
    ├── Hipparcos_Below_6.0.csv           ← 별 카탈로그 (4992개, mag≤6.0)
    └── star_catalog_kvector.mat          ← K-vector 인덱스 카탈로그
```

### 1.3 시뮬레이션 파이프라인

```
입력: (RA, DEC, Roll) [deg] + sensor_params
  │
  ├─[1] FOV 계산 ─────────────── §2
  ├─[2] 회전행렬 생성 (Z-X-Z) ── §3.2
  ├─[3] 별 카탈로그 검색 ──────── §2.2
  ├─[4] 좌표 변환 ────────────── §3
  │     천구(RA,DEC) → ECI 벡터 → 센서 좌표 → 핀홀 투영 → 픽셀
  ├─[5] Pogson 플럭스 계산 ────── §4
  ├─[6] Gaussian PSF 렌더링 ──── §5
  ├─[7] 모션블러 + 롤링셔터 ──── §6
  ├─[8] 노이즈 체인 (7종) ────── §7
  │     PRNU → Dark → Stray → DSNU → Shot → CR → Gain → Read → ADC
  ├─[9] Bayer RGGB 패턴 생성 ── §8
  │
  ▼
출력: gray_img [uint16], bayer_img [uint16], star_info [struct]
```

---

## 2. FOV(시야각) 계산

> 소스: `core/simulate_star_image_realistic.m`

### 2.1 핀홀 모델 FOV

$$\text{FOV}_x = 2 \arctan\!\left(\frac{\mu \cdot N_x / 2}{f}\right), \quad \text{FOV}_y = 2 \arctan\!\left(\frac{\mu \cdot N_y / 2}{f}\right)$$

| 기호 | 의미 | 단위 | 예시 (OV4689) |
|------|------|------|--------------|
| $\mu$ | 픽셀 크기 | m | $2 \times 10^{-6}$ |
| $N_x, N_y$ | 수평/수직 픽셀 수 | px | 1280 × 720 |
| $f$ | 초점거리 | m | 0.01042 |

**수치 예 (OV4689 + 10.42 mm 렌즈):**

$$\text{FOV}_x = 2 \arctan\!\left(\frac{2\times10^{-6} \times 640}{0.01042}\right) = 2 \arctan(0.1228) \approx 14.02°$$

$$\text{FOV}_y = 2 \arctan\!\left(\frac{2\times10^{-6} \times 360}{0.01042}\right) \approx 7.91°$$

$$\text{FOV}_{\text{diag}} = \sqrt{14.02^2 + 7.91^2} \approx 16.1°$$

### 2.2 FOV 검색 영역 (cos(DEC) 보정)

별 카탈로그에서 FOV 내 별을 검색할 때, 천구 좌표의 극 수렴(pole convergence) 효과를 보정해야 한다.

검색 반경 (대각 FOV의 절반):

$$R = \frac{1}{2}\sqrt{\text{FOV}_{x,\text{rad}}^2 + \text{FOV}_{y,\text{rad}}^2} \quad [\text{rad}]$$

RA 검색 범위 (극 보정 포함):

$$\alpha_{\min} = \alpha_0 - \frac{R}{\cos \delta_0}, \quad \alpha_{\max} = \alpha_0 + \frac{R}{\cos \delta_0}$$

DEC 검색 범위:

$$\delta_{\min} = \delta_0 - R, \quad \delta_{\max} = \delta_0 + R$$

**물리적 의미:** DEC = 60°에서 1° 각거리에 해당하는 RA 범위 = $1/\cos(60°) = 2°$. 이 보정 없이는 극 부근에서 별을 놓치게 된다.

---

## 3. 좌표 변환

> 소스: `core/simulate_star_image_realistic.m` — `create_rotation_matrix()`

### 3.1 천구 좌표 → ECI 단위 벡터

적경(RA) $\alpha$와 적위(DEC) $\delta$로부터 3D 방향 벡터:

$$\mathbf{d}_{\text{ECI}} = \begin{bmatrix} \cos\alpha \cos\delta \\ \sin\alpha \cos\delta \\ \sin\delta \end{bmatrix}, \quad \|\mathbf{d}\| = 1$$

### 3.2 Z-X-Z 오일러 회전행렬 (ECI → 센서)

3단계 회전으로 ECI 프레임에서 센서 Body 프레임으로 변환한다.

**오프셋 적용:**

$$\alpha_{\text{exp}} = \alpha - \frac{\pi}{2}, \quad \delta_{\text{exp}} = \delta + \frac{\pi}{2}$$

- $-\pi/2$ (RA): 춘분점(RA=0)과 센서 기준축 간 90° 차이 보정
- $+\pi/2$ (DEC): DEC=0(적도)에서 광축이 적도면을 향하도록 보정

**Step 1 — Z축 회전 (RA):**

$$M_1 = \begin{bmatrix} \cos\alpha_{\text{exp}} & -\sin\alpha_{\text{exp}} & 0 \\ \sin\alpha_{\text{exp}} & \cos\alpha_{\text{exp}} & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**Step 2 — X축 회전 (DEC):**

$$M_2 = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\delta_{\text{exp}} & -\sin\delta_{\text{exp}} \\ 0 & \sin\delta_{\text{exp}} & \cos\delta_{\text{exp}} \end{bmatrix}$$

**Step 3 — Z축 회전 (Roll):**

$$M_3 = \begin{bmatrix} \cos\phi & -\sin\phi & 0 \\ \sin\phi & \cos\phi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**최종 회전행렬:**

$$M = M_1 \cdot M_2 \cdot M_3 \quad (3 \times 3 \text{ 직교행렬, } M^{-1} = M^T)$$

**ECI → 센서 좌표 변환:**

$$\begin{bmatrix} X_s \\ Y_s \\ Z_s \end{bmatrix} = M^T \cdot \mathbf{d}_{\text{ECI}}$$

$Z_s > 0$이면 별이 센서 전방에 위치.

### 3.3 핀홀 카메라 투영

유사삼각형(similar triangles)으로부터:

$$x_{\text{img}} = f \cdot \frac{X_s}{Z_s} \quad [\text{m}], \quad y_{\text{img}} = f \cdot \frac{Y_s}{Z_s} \quad [\text{m}]$$

### 3.4 픽셀 좌표 변환

물리 좌표 [m] → 픽셀 좌표 (좌상단 원점, Y축 반전):

$$x_{\text{px}} = \frac{N_x}{2} + \frac{x_{\text{img}}}{\mu}, \quad y_{\text{px}} = \frac{N_y}{2} - \frac{y_{\text{img}}}{\mu}$$

```
물리 좌표계 (센서면):        이미지 좌표계 (MATLAB):
      Y↑                    (1,1)──────→ col (+x)
       │                         │
  ─────┼──────→ X                ↓
       │                       row (+y)
```

Y축 반전 이유: 물리 좌표계에서 Y+는 위쪽이지만, 이미지 좌표계에서 row+는 아래쪽이다.

### 3.5 IFOV (순간 시야각)

단일 픽셀의 각도 분해능:

$$\text{IFOV} = \arctan\!\left(\frac{\mu}{f}\right)$$

OV4689 예: $\arctan(2 \times 10^{-6} / 0.01042) \approx 0.011°/\text{pixel}$

---

## 4. 항성 측광: Pogson 공식

> 소스: `core/simulate_star_image_realistic.m`

### 4.1 등급–플럭스 변환 (Pogson's Law)

$$F = F_{\text{ref}} \cdot 10^{-0.4 \,(m - m_{\text{ref}})} \quad [\text{photons/s}]$$

| 기호 | 의미 | 값 |
|------|------|-----|
| $F_{\text{ref}}$ | 기준 플럭스 | 지상 96, 우주 $\approx 171$ photons/s |
| $m_{\text{ref}}$ | 기준 등급 | 6.0 |
| $m$ | 별의 겉보기 등급 | Hipparcos 카탈로그 값 |

**Pogson 법칙의 물리적 의미:** 5등급 차이 = 밝기 100배. 따라서 1등급 차이 = $10^{0.4} \approx 2.512$배.

**우주 환경 보정:**

$$F_{\text{space}} = \frac{F_{\text{ground}}}{\tau_{\text{atm}}} = \frac{96}{0.56} \approx 171 \; [\text{photons/s at } m = 6.0]$$

여기서 $\tau_{\text{atm}} = 0.56$은 지상 대기 투과율. 우주에서는 대기 흡수가 없으므로 더 많은 광자 수신.

**등급별 플럭스 예시 (우주 환경):**

| 등급 | 상대 플럭스 (6등급=1) | 절대 플럭스 [photons/s] |
|------|---------------------|----------------------|
| 0 | $10^{2.4} \approx 251$ | ~42,900 |
| 3 | $10^{1.2} \approx 15.8$ | ~2,710 |
| 6 | 1 | 171 |
| 6.5 | $10^{-0.2} \approx 0.63$ | ~108 |

### 4.2 신호 체인: 광자 → 전자 → ADU

$$N_e = F \cdot t_{\text{exp}} \cdot \eta_{\text{QE}} \quad [\text{electrons}]$$

$$S_{\text{ADU}} = N_e \cdot G_a \cdot G_d \quad [\text{ADU}]$$

| 기호 | 의미 | 단위 | 예시 |
|------|------|------|------|
| $t_{\text{exp}}$ | 노출 시간 | s | 0.022 |
| $\eta_{\text{QE}}$ | 양자 효율 | – | 0.50 |
| $G_a$ | 아날로그 게인 | × | 16 |
| $G_d$ | 디지털 게인 | × | 1.0 |

---

## 5. Gaussian PSF (점확산함수)

> 소스: `core/simulate_star_image_realistic.m` — `draw_star_psf()`, `core/get_sensor_preset.m`

### 5.1 2D Gaussian PSF 모델

별은 점광원(point source)이므로, 광학계의 회절과 수차에 의해 이미지에서 유한한 크기의 반점(PSF)으로 나타난다.

$$I(x, y) = A \cdot \exp\!\left(-\frac{(x - x_c)^2 + (y - y_c)^2}{2\sigma^2}\right) \quad [\text{ADU/pixel}]$$

**에너지 보존 정규화:**

$$\iint_{-\infty}^{\infty} I(x,y)\,dx\,dy = 2\pi\sigma^2 A = S_{\text{total}}$$

$$\therefore \; A = \frac{S_{\text{total}}}{2\pi\sigma^2} \quad [\text{ADU/pixel}]$$

| 기호 | 의미 | 단위 |
|------|------|------|
| $x_c, y_c$ | 별 중심 좌표 (서브픽셀) | pixel |
| $\sigma$ | PSF 표준편차 | pixel |
| $S_{\text{total}}$ | 별의 총 신호 (적분 플럭스) | ADU |

### 5.2 PSF Sigma: Airy Disk 유도

> 소스: `core/get_sensor_preset.m`

원형 개구(aperture)에 의한 회절 한계 (Airy disk):

$$r_{\text{Airy}} = 1.22 \cdot \lambda \cdot F_{\#} \quad [\text{m}]$$

$$\sigma_{\text{PSF}} = \max\!\left(0.8, \; \frac{r_{\text{Airy}}}{\mu}\right) \quad [\text{pixels}]$$

| 기호 | 의미 | 값 |
|------|------|-----|
| $\lambda$ | 기준 파장 (V-band) | 550 nm |
| $F_{\#}$ | 렌즈 F-number | 렌즈별 상이 |
| 0.8 px | 최소 sigma 하한 | 서브픽셀 PSF 방지 |

**렌즈별 Airy 반경 예:**

| 렌즈 | $F_{\#}$ | $r_{\text{Airy}}$ [nm] | OV4689 (2 μm) $\sigma$ [px] |
|------|---------|---------------------|---------------------------|
| 6 mm f/1.2 | 1.2 | 805 | max(0.8, 0.40) = **0.80** |
| 16 mm f/1.4 | 1.4 | 939 | max(0.8, 0.47) = **0.80** |
| 25 mm f/2.0 | 2.0 | 1,342 | max(0.8, 0.67) = **0.80** |

### 5.3 FWHM (반치전폭)

$$\text{FWHM} = 2\sqrt{2\ln 2} \cdot \sigma \approx 2.355 \cdot \sigma \quad [\text{pixels}]$$

$\sigma = 1.2$ px 일 때: FWHM $\approx 2.8$ px, 예상 별 면적 $\approx \pi(1.4)^2 \approx 6$ pixels.

### 5.4 렌더링 윈도우

$$\text{Window} = \pm 6\sigma \quad \text{pixels}$$

$6\sigma$ 범위는 2D Gaussian 에너지의 99.99%를 포함한다.

---

## 6. 모션블러 & 롤링셔터

> 소스: `core/simulate_star_image_realistic.m`

### 6.1 모션블러 (Global Shutter & Rolling Shutter 공통)

위성의 각속도 $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]$ [rad/s]에서, 노출 시간 $t_{\text{exp}}$ 동안 별 이미지가 센서면에서 이동하는 거리:

$$\Delta x_{\text{blur}} = \omega_y \cdot \frac{f}{\mu} \cdot t_{\text{exp}} \quad [\text{pixels}]$$

$$\Delta y_{\text{blur}} = -\omega_x \cdot \frac{f}{\mu} \cdot t_{\text{exp}} \quad [\text{pixels}]$$

$$L_{\text{blur}} = \sqrt{\Delta x_{\text{blur}}^2 + \Delta y_{\text{blur}}^2} \quad [\text{pixels}]$$

| 기호 | 의미 | 단위 |
|------|------|------|
| $f/\mu$ | 각도→픽셀 변환 계수 | pixel/rad |
| $\omega_y$ | Yaw 각속도 → x 방향 이동 | rad/s |
| $\omega_x$ | Pitch 각속도 → y 방향 이동 (부호 반전) | rad/s |

**구현:** PSF를 노출 궤적을 따라 $n = \max(3, \lfloor 2L_{\text{blur}} \rceil)$개 등분점에서 누적 렌더링.

### 6.2 롤링셔터 (RS센서 전용)

Rolling Shutter는 행(row)별로 노출 시작 시각이 다르다.

행 $r$ (0-indexed)의 노출 시작 시간:

$$t_r = \frac{r}{N_y} \cdot t_{\text{readout}} \quad [\text{s}]$$

행 $r$에서 별 중심의 추가 이동:

$$\Delta x_{\text{RS}}(r) = \omega_y \cdot \frac{f}{\mu} \cdot t_r \quad [\text{pixels}]$$

$$\Delta y_{\text{RS}}(r) = -\omega_x \cdot \frac{f}{\mu} \cdot t_r \quad [\text{pixels}]$$

### 6.3 결합 모드 (RS + Motion Blur)

롤링셔터와 모션블러가 동시 적용될 때, 각 행 $r$마다:
1. RS에 의한 별 중심 이동 적용: $(x_c + \Delta x_{\text{RS}}(r),\; y_c + \Delta y_{\text{RS}}(r))$
2. 이동된 중심을 기준으로 모션블러 궤적을 따라 PSF 누적

결과적으로 별 이미지는 RS에 의한 행별 시프트와 모션블러에 의한 선분 번짐이 중첩된다.

---

## 7. 노이즈 모델 (7종)

> 소스: `core/simulate_star_image_realistic.m`

### 노이즈 체인 순서 (변경 금지)

```
광자 입사
 ↓ QE 변환 (광자 → 전자)
 ↓ [7.1] PRNU (승산적 FPN)
 ↓ 배경 전자 합산:
 │  ├─ [7.2] Dark Current (Arrhenius)
 │  ├─ [7.3] Stray Light
 │  └─ [7.4] DSNU (가산적 FPN)
 ↓ [7.5] Shot Noise (Poisson)
 ↓ [7.6] Cosmic Ray
 ↓ 게인 적용 (전자 → ADU)
 ↓ [7.7] Read Noise (Gaussian)
 ↓ ADC 클램핑
```

### 7.1 PRNU (Photo Response Non-Uniformity)

픽셀 간 감도 차이에 의한 승산적(multiplicative) 고정 패턴 노이즈.

$$S_{\text{PRNU}}(r,c) = S_{\text{signal}}(r,c) \cdot P(r,c)$$

$$P(r,c) = 1 + \sigma_{\text{PRNU}} \cdot \mathcal{N}(0,1), \quad \sigma_{\text{PRNU}} = 0.01 \; (1\%)$$

고정 시드(seed=43) 사용 → 동일 센서에서 매 프레임 동일한 공간 패턴.

### 7.2 다크 전류 (Arrhenius 온도 모델)

반도체의 열적 여기(thermal excitation)에 의한 암전류. 온도에 지수적으로 의존한다.

$$I_{\text{dark}}(T) = I_{\text{ref}} \cdot 2^{(T - T_{\text{ref}}) / T_{\text{double}}} \quad [\text{e}^-/\text{px/s}]$$

$$N_{\text{dark}} = I_{\text{dark}}(T) \cdot t_{\text{exp}} \quad [\text{e}^-/\text{pixel}]$$

| 파라미터 | 기본값 | 의미 |
|---------|-------|------|
| $I_{\text{ref}}$ | 0.1 e⁻/px/s | 기준 온도(25°C)에서의 다크 전류 |
| $T_{\text{ref}}$ | 25°C | 기준 온도 |
| $T_{\text{double}}$ | 6.5°C | 다크 전류가 2배가 되는 온도 간격 |

**수치 예 (우주, $T = -20°C$):**

$$I_{\text{dark}}(-20°C) = 0.1 \cdot 2^{(-20 - 25)/6.5} = 0.1 \cdot 2^{-6.92} \approx 8.2 \times 10^{-4} \; [\text{e}^-/\text{px/s}]$$

→ 지상(25°C) 대비 약 **122배 감소**. 이것이 별센서를 저온에서 운용하는 핵심 이유.

### 7.3 스트레이라이트 (Stray Light)

태양, 지구, 달 반사광 등에 의한 균일 배경 광전자.

$$N_{\text{stray}} = I_{\text{stray}} \cdot t_{\text{exp}} \quad [\text{e}^-/\text{pixel}]$$

바플(baffle) 성능에 따라 $I_{\text{stray}} = 0 \sim 10$ e⁻/px/s. 기본값 = 0 (이상적 차폐).

### 7.4 DSNU (Dark Signal Non-Uniformity)

다크 전류의 픽셀 간 변동에 의한 가산적(additive) 고정 패턴 노이즈.

$$D(r,c) = \sigma_{\text{DSNU}} \cdot \mathcal{N}(0,1) \cdot \frac{I_{\text{dark}}(T)}{I_{\text{ref}}}$$

$$N_{\text{DSNU}}(r,c) = D(r,c) \cdot t_{\text{exp}} \quad [\text{e}^-]$$

기본 $\sigma_{\text{DSNU}} = 0.5$ e⁻/s RMS. 고정 시드(seed=42). 온도에 비례하여 스케일.

### 7.5 Shot Noise (Poisson)

광전 변환의 양자적 불확실성. 전자 수 자체가 Poisson 분포를 따른다.

$$N_{\text{total}} = N_{\text{signal}} \cdot P_{\text{PRNU}} + N_{\text{dark}} + N_{\text{stray}} + N_{\text{DSNU}}$$

$$N_{\text{noisy}} \sim \text{Poisson}\!\left(\max(0,\; N_{\text{total}})\right)$$

Poisson 분포의 성질: $\langle N \rangle = \text{Var}(N) = N_{\text{total}}$, 따라서 $\sigma_{\text{shot}} = \sqrt{N_{\text{total}}}$.

### 7.6 우주방사선 (Cosmic Ray)

LEO 530 km 궤도에서 고에너지 입자가 센서에 충돌하여 전자를 생성한다.

**히트 수 (Poisson 과정):**

$$A_{\text{sensor}} = (\mu \cdot N_x \cdot 100) \times (\mu \cdot N_y \cdot 100) \quad [\text{cm}^2]$$

$$\mu_{\text{hits}} = R_{\text{CR}} \cdot A_{\text{sensor}} \cdot t_{\text{exp}}$$

$$n_{\text{hits}} \sim \text{Poisson}(\mu_{\text{hits}})$$

| 파라미터 | 기본값 | 의미 |
|---------|-------|------|
| $R_{\text{CR}}$ | 5 hits/cm²/s | LEO 평균 히트율 (SAA 외) |
| $E_{\text{CR}}$ | 1000 e⁻/hit | 평균 충돌 에너지 |

**OV4689 (1280×720, 22ms) 예:**

$$A = (2\times10^{-4} \cdot 1280)(2\times10^{-4} \cdot 720) \approx 0.037 \; \text{cm}^2$$

$$\mu_{\text{hits}} = 5 \times 0.037 \times 0.022 \approx 0.004 \; \text{hits/frame}$$

**3×3 Gaussian 클러스터 분포:**

각 히트의 에너지 $E = E_{\text{CR}} \cdot (0.5 + U[0,1])$ 이며, 충돌 지점 주변 3×3 영역에 분산:

$$\Delta N(dr, dc) = E \cdot \exp\!\left(-\frac{dr^2 + dc^2}{0.8}\right), \quad dr, dc \in \{-1, 0, 1\}$$

$0.8 = 2 \times 0.4^2$ → 유효 $\sigma \approx 0.63$ pixels (compact ionization track).

### 7.7 읽기 노이즈 (Read Noise)

센서 읽기 회로의 열적 노이즈. ADU 변환 후 Gaussian으로 적용.

$$S_{\text{out}} = N_{\text{noisy}} \cdot G_{\text{total}} + \sigma_r \cdot G_{\text{total}} \cdot \mathcal{N}(0,1) \quad [\text{ADU}]$$

여기서 $\sigma_r$는 읽기 노이즈 [e⁻ RMS], $G_{\text{total}} = G_a \cdot G_d$.

### 7.8 ADC 클램핑

$$S_{\text{final}} = \text{clip}\!\left(S_{\text{out}}, \; 0, \; 2^{B_{\text{ADC}}} - 1\right)$$

| ADC Bits | 최대값 |
|----------|-------|
| 8 | 255 |
| 10 | 1,023 |
| 12 | 4,095 |

---

## 8. Bayer 패턴 (RGGB)

> 소스: `core/simulate_star_image_realistic.m`

### 8.1 RGGB 레이아웃

```
         col 짝수(c_idx=0)   col 홀수(c_idx=1)
row 짝수(r_idx=0):    R              Gr
row 홀수(r_idx=1):    Gb              B
```

위치 인덱스 계산 (MATLAB 1-based):

$$r_{\text{idx}} = (r-1) \bmod 2, \quad c_{\text{idx}} = (c-1) \bmod 2$$

$$\text{pos\_idx} = r_{\text{idx}} \times 2 + c_{\text{idx}} \in \{0(\text{R}),\; 1(\text{Gr}),\; 2(\text{Gb}),\; 3(\text{B})\}$$

### 8.2 채널 감도

Bayer 필터의 컬러별 투과율 차이를 반영:

$$\text{bayer}(r,c) = \text{gray}(r,c) \times s_{\text{channel}}$$

| 센서 | $s_R$ | $s_G$ | $s_B$ |
|------|-------|-------|-------|
| OV4689 (기본) | 0.916 | 1.000 | 0.854 |
| IMX585 (Starvis 2) | 0.920 | 1.000 | 0.780 |
| OV9281 (모노) | 1.000 | 1.000 | 1.000 |

OV4689의 감도 비율 근거: R/G = 46.7%/51.1% = 0.916, B/G = 43.6%/51.1% = 0.854.

---

## 9. Bayer→Gray 변환 5종

> 소스: `core/bayer_to_gray_direct.m`, `core/bayer_to_rgb_cfa.m`, `core/rgb_to_gray_fpga.m`

### 9.1 방법 비교 요약

| 방법 | 공식 | 출력 크기 | SNR | 용도 |
|------|------|----------|-----|------|
| `raw` | 그대로 통과 | $H \times W$ | 기준 | 최고 속도 |
| `binning` | $(R + G_r + G_b + B)/4$ | $H/2 \times W/2$ | **+6 dB** | FPGA, 저조도 |
| `green` | $G$ 채널 + 십자 보간 | $H \times W$ | — | 휘도 근사 |
| `weighted` | $(R + 2G + B)/4$ | $H \times W$ | — | FPGA 파이프라인 |
| `optimal` | $w_R R + w_G G + w_B B$ | $H \times W$ | **최고** | SNR 최대화 |

### 9.2 Method 1: `raw` (패스스루)

$$Y(r,c) = \text{bayer}(r,c)$$

변환 없이 Bayer 값을 그대로 사용. 최고 속도이나 Bayer 패턴에 의한 공간 아티팩트 존재.

### 9.3 Method 2: `binning` (2×2 블록 평균)

$$Y(i,j) = \frac{R + G_r + G_b + B}{4} = \frac{R + 2G + B}{4}$$

각 2×2 RGGB 블록을 1개 픽셀로 합산. 출력 해상도는 $H/2 \times W/2$.

**SNR 향상 원리:**

$$\text{SNR}_{\text{out}} = \text{SNR}_{\text{in}} \cdot \sqrt{N}, \quad N = 4$$

$$\Delta\text{SNR} = 20 \log_{10}(\sqrt{4}) = 20 \log_{10}(2) \approx +6 \; \text{dB}$$

4개 독립 샘플 평균 → 노이즈 $\sqrt{4} = 2$배 감소.

### 9.4 Method 3: `green` (Green 채널 추출 + 십자 보간)

Green 픽셀 위치: 값을 그대로 사용.

R 또는 B 픽셀 위치: 인접 4개 Green 값의 십자 평균으로 보간.

$$G_{\text{interp}}(r,c) = \frac{G_{\text{위}} + G_{\text{아래}} + G_{\text{왼}} + G_{\text{오른}}}{4}$$

경계 처리: `replicate` 패딩 (가장자리 픽셀 복제).

### 9.5 Method 4: `weighted` (FPGA 파이프라인 통합)

CFA 디모자이킹과 RGB→Gray를 단일 패스로 통합. 3×3 윈도우 기반.

$$Y = \frac{R + 2G + B}{4}$$

각 pos_idx에 따라 R, G, B를 로컬 이웃에서 추정:

| pos\_idx | R | G | B |
|----------|---|---|---|
| 0 (R) | center | cross avg (4개) | diagonal avg (4개) |
| 1 (Gr) | left+right avg | center | top+bottom avg |
| 2 (Gb) | top+bottom avg | center | left+right avg |
| 3 (B) | diagonal avg (4개) | cross avg (4개) | center |

### 9.6 Method 5: `optimal` (SNR 최대화 가중치)

$$Y = w_R \cdot R + w_G \cdot G + w_B \cdot B$$

$$w_R = 0.4544, \quad w_G = 0.3345, \quad w_B = 0.2111 \quad \left(\sum w_i = 1\right)$$

**이론적 근거: Inverse Variance Weighting**

최적 가중치는 각 채널의 SNR 기여를 최대화하도록 도출된다:

$$w_i \propto \frac{S_i}{\sigma_i^2}$$

| 노이즈 지배 영역 | $\sigma_i^2$ | 가중치 |
|----------------|------------|-------|
| Shot noise (밝은 별) | $\sigma^2 \approx S$ | $w_i \propto 1$ (균등) |
| Read noise (어두운 별) | $\sigma^2 \approx \sigma_r^2$ | $w_i \propto S_i$ (신호 비례) |

도출 조건: OV4689, 우주 환경(-20°C), 전 스펙트럼 등급 가중 평균.

### 9.7 CFA Bilinear 디모자이킹

> 소스: `core/bayer_to_rgb_cfa.m` — FPGA `cfa.cpp` 재현

3×3 윈도우 $w$에서 각 위치별 R, G, B 복원 공식:

**pos\_idx = 0 (R 위치):**

$$R = w_{2,2}, \quad G = \frac{w_{1,2} + w_{2,1} + w_{2,3} + w_{3,2}}{4}, \quad B = \frac{w_{1,1} + w_{1,3} + w_{3,1} + w_{3,3}}{4}$$

**pos\_idx = 1 (Gr 위치):**

$$R = \frac{w_{2,1} + w_{2,3}}{2}, \quad G = w_{2,2}, \quad B = \frac{w_{1,2} + w_{3,2}}{2}$$

**pos\_idx = 2 (Gb 위치):**

$$R = \frac{w_{1,2} + w_{3,2}}{2}, \quad G = w_{2,2}, \quad B = \frac{w_{2,1} + w_{2,3}}{2}$$

**pos\_idx = 3 (B 위치):**

$$R = \frac{w_{1,1} + w_{1,3} + w_{3,1} + w_{3,3}}{4}, \quad G = \frac{w_{1,2} + w_{2,1} + w_{2,3} + w_{3,2}}{4}, \quad B = w_{2,2}$$

경계 처리: `replicate` 패딩 (FPGA line buffer 동작과 동일).

### 9.8 FPGA RGB→Gray

> 소스: `core/rgb_to_gray_fpga.m` — FPGA `rgb2gray.cpp` 재현

$$Y = \frac{R + 2G + B}{4} = 0.25R + 0.50G + 0.25B$$

**FPGA 구현:** 곱셈기 없이 비트 연산만 사용.
- $2G$ = 1비트 좌측 시프트
- $\div 4$ = 2비트 우측 시프트

**ITU-R BT.601 표준 대비:**

| 채널 | BT.601 | FPGA 근사 | 오차 |
|------|--------|----------|------|
| R | 0.299 | 0.250 | −0.049 |
| G | 0.587 | 0.500 | −0.087 |
| B | 0.114 | 0.250 | +0.136 |

---

## 10. 별 검출 알고리즘

> 소스: `core/detect_stars_simple.m`

### 10.1 배경 추정

$$\text{BG} = \text{median}(\text{img}[:]) \quad [\text{ADU}]$$

중앙값(median) 사용 이유: 별 픽셀은 전체의 0.1% 미만이므로, 극단값에 강건한 추정.

### 10.2 이진화 (Thresholding)

$$\text{binary}(r,c) = \begin{cases} 1 & \text{if } \text{img}(r,c) > \text{BG} + T \\ 0 & \text{otherwise} \end{cases}$$

기본 $T = 20$ ADU (gui에서 조절 가능).

### 10.3 Connected Component Labeling (CCL)

8-연결(상하좌우 + 대각 4방향)로 인접 픽셀을 그룹화. MATLAB `bwconncomp` 사용.

**면적 필터:**

$$A_{\text{blob}} \geq A_{\min} \quad (\text{기본 } A_{\min} = 3 \text{ pixels})$$

PSF $\sigma=1.2$ px에서 예상 별 면적: FWHM $\approx 2.8$ px → $\pi \cdot (1.4)^2 \approx 6$ pixels.

$A_{\min} = 3$은 노이즈 핫픽셀(1~2 pixel) 필터링과 faint star 검출의 균형점.

### 10.4 CoG Centroid (밝기 가중 중심)

$$x_c = \frac{\sum_{(x,y) \in \Omega} I(x,y) \cdot x}{\sum_{(x,y) \in \Omega} I(x,y)}, \quad y_c = \frac{\sum_{(x,y) \in \Omega} I(x,y) \cdot y}{\sum_{(x,y) \in \Omega} I(x,y)}$$

$\Omega$: 해당 connected component에 속하는 픽셀 집합.

MATLAB `regionprops(..., img, 'Centroid')` 구현 → 서브픽셀(sub-pixel) 정밀도 달성.

---

## 11. SNR 지표

### 11.1 Peak SNR (단일 이미지 기반)

> 소스: `core/calculate_peak_snr.m`

$$\text{Peak SNR} = 20 \cdot \log_{10}\!\left(\frac{S_{\text{peak}}}{\sigma_{\text{noise}}}\right) \quad [\text{dB}]$$

$$S_{\text{peak}} = \max(\text{img}) - \text{median}(\text{img})$$

$$\sigma_{\text{noise}} = \text{std}(\text{img}[\text{img} < \text{BG} + 10])$$

**20×log₁₀ 사용 이유:** 픽셀 값은 진폭(amplitude) 단위이므로 $20 \log_{10}$ 사용. 파워 단위라면 $10 \log_{10}$.

배경 픽셀만으로 노이즈 표준편차를 계산하여, 별 신호에 의한 bias를 제거.

$\sigma_{\text{noise}} < 0.1$이면 100 dB 반환 (무잡음 이미지 sentinel).

### 11.2 Reference SNR (방법 비교용)

> 소스: `core/calculate_snr.m`

$$\text{SNR} = 10 \cdot \log_{10}\!\left(\frac{P_{\text{signal}}}{P_{\text{noise}}}\right) \quad [\text{dB}]$$

$$P_{\text{signal}} = \text{mean}(\text{reference}^2) \quad [\text{ADU}^2]$$

$$P_{\text{noise}} = \text{mean}((\text{measured} - \text{reference})^2) \quad [\text{ADU}^2]$$

**10×log₁₀ 사용 이유:** 분자/분모 모두 이미 제곱된 파워(power) 단위.

Bayer→Gray 변환 방법의 품질을 정량 비교할 때 사용. 크기가 다른 경우(binning) bilinear 보간으로 reference를 리사이즈.

### 11.3 해석 기준

| Peak SNR | 해석 |
|----------|------|
| > 30 dB | 우수 — 안정적 검출 |
| 20–30 dB | 양호 — 검출 가능 |
| < 15 dB | 불량 — 검출 불안정 |

---

## 12. Centroid 정확도 평가

> 소스: `core/evaluate_centroid_accuracy.m`

### 12.1 Nearest Neighbor 매칭

각 true star 위치 $\mathbf{t}_i$에 대해 가장 가까운 detected star를 매칭:

$$d_i = \min_j \sqrt{(x_{\text{det},j} - x_{\text{true},i})^2 + (y_{\text{det},j} - y_{\text{true},i})^2}$$

매칭 조건: $d_i < r_{\text{match}}$ (기본 5.0 pixels).

### 12.2 RMS 오차

$$\text{RMS} = \sqrt{\frac{1}{K} \sum_{k=1}^{K} e_k^2} \quad [\text{pixels}]$$

$K$ = 성공적으로 매칭된 별의 수, $e_k$ = 각 매칭 쌍의 Euclidean 거리.

### 12.3 목표 정확도

별센서 목표 정확도: **0.015° = 54 arcsec**

OV4689 + 10.42 mm에서 픽셀 각도 분해능:

$$\theta_{\text{1px}} = \arctan\!\left(\frac{\mu}{f}\right) = \arctan\!\left(\frac{2 \times 10^{-6}}{0.01042}\right) \approx 0.011°/\text{pixel}$$

**목표 RMS:**

$$\text{RMS}_{\text{target}} = \frac{0.015°}{0.011°/\text{px}} \approx 1.4 \; \text{pixels}$$

### 12.4 성능 기준 요약

| 메트릭 | 목표 | 설명 |
|--------|------|------|
| Centroid RMS | < 1.4 pixel | 0.015° 정확도 달성 |
| Peak SNR | > 20 dB | 별 검출 최소 조건 |
| 검출 별 수 | $\geq 3$ | 자세 결정(attitude determination) 최소 조건 |

---

## 13. 부록: 센서/렌즈 데이터베이스

> 소스: `core/get_sensor_preset.m`

### 13.1 지원 센서 18종

| # | 센서 | 제조사 | 셔터 | 픽셀 [μm] | QE | FW [ke⁻] | Read [e⁻] | Dark @25°C [e⁻/px/s] | ADC [bit] |
|---|------|--------|------|----------|-----|---------|----------|---------------------|----------|
| 1 | IMX585 | Sony Starvis 2 | RS | 2.9 | 0.90 | 47 | 1.0 | 0.05 | 12 |
| 2 | IMX678 | Sony Starvis 2 | RS | 2.0 | 0.83 | 11.3 | 1.0 | 0.08 | 12 |
| 3 | IMX662 | Sony Starvis 2 | RS | 2.9 | 0.91 | 38 | 1.2 | 0.08 | 12 |
| 4 | IMX485 | Sony Starvis 1 | RS | 2.9 | 0.85 | 13 | 1.5 | 0.15 | 12 |
| 5 | IMX462 | Sony Starvis 1 | RS | 2.9 | 0.80 | 15 | 2.5 | 0.15 | 12 |
| 6 | IMX174 | Sony Pregius S1 | GS | 5.86 | 0.77 | 32 | 5.0 | 0.10 | 12 |
| 7 | IMX249 | Sony Pregius S1 | GS | 5.86 | 0.77 | 32 | 5.0 | 0.10 | 12 |
| 8 | IMX264 | Sony Pregius S2 | GS | 3.45 | 0.72 | 10 | 3.5 | 0.10 | 12 |
| 9 | IMX250 | Sony Pregius S2 | GS | 3.45 | 0.72 | 10 | 3.5 | 0.10 | 12 |
| 10 | IMX252 | Sony Pregius S2 | GS | 3.45 | 0.72 | 10 | 3.5 | 0.10 | 12 |
| 11 | IMX296 | Sony | GS | 3.45 | 0.65 | 10 | 2.2 | 0.10 | 10 |
| 12 | IMX477 | Sony | RS | 1.55 | 0.78 | 4.5 | 3.3 | 0.05 | 12 |
| 13 | IMX219 | Sony | RS | 1.12 | 0.65 | 2.8 | 4.5 | 0.15 | 10 |
| 14 | CMV4000 | ams-OSRAM | GS | 5.5 | 0.50 | 13.5 | 10.0 | 0.50 | 10 |
| 15 | CMV2000 | ams-OSRAM | GS | 5.5 | 0.50 | 13.5 | 10.0 | 0.50 | 10 |
| 16 | OV9281 | OmniVision | GS | 3.0 | 0.55 | 6 | 5.0 | 0.20 | 10 |
| 17 | MT9P031 | onsemi | RS | 2.2 | 0.50 | 6 | 8.0 | 0.30 | 12 |
| 18 | AR0134 | onsemi | GS | 3.75 | 0.55 | 8 | 6.0 | 0.30 | 12 |

- **GS** = Global Shutter, **RS** = Rolling Shutter
- **QE** = Quantum Efficiency (V-band 기준)
- **FW** = Full Well Capacity [kilo-electrons]

### 13.2 지원 렌즈 8종

| # | 명칭 | 초점거리 | F# | 마운트 |
|---|------|---------|-----|--------|
| 1 | 6mm f/1.2 | 6 mm | 1.2 | CS |
| 2 | 8mm f/1.2 | 8 mm | 1.2 | CS |
| 3 | 12mm f/1.4 | 12 mm | 1.4 | CS |
| 4 | 16mm f/1.4 | 16 mm | 1.4 | CS |
| 5 | 25mm f/2.0 | 25 mm | 2.0 | CS |
| 6 | 8-50mm @50mm f/1.4 | 50 mm | 1.4 | C |
| 7 | 8-50mm @8mm f/1.4 | 8 mm | 1.4 | C |
| 8 | 8-50mm @25mm f/1.4 | 25 mm | 1.4 | C |

### 13.3 물리 상수 및 기준값

| 상수 | 값 | 출처/의미 |
|------|-----|---------|
| 기준 광자 플럭스 (지상) | 96 photons/s @ V=6 | 정규화된 기준 |
| 기준 광자 플럭스 (우주) | 171 photons/s @ V=6 | = 96/0.56 (대기 보정) |
| 대기 투과율 | 0.56 | 지상 vs 우주 보정 |
| 기준 등급 | 6.0 | Hipparcos 카탈로그 한계 |
| Arrhenius $T_{\text{double}}$ | 6.5°C | CMOS 다크 전류 배증 온도 |
| Airy disk 기준 파장 | 550 nm | V-band 중심 |
| 우주방사선율 (LEO) | 5 hits/cm²/s | 530 km SSO, SAA 외 평균 |
| 우주방사선 에너지 | 1000 e⁻/hit | 평균 히트 에너지 |
| PRNU 기본값 | 1% RMS | 승산적 FPN |
| DSNU 기본값 | 0.5 e⁻/s RMS | 가산적 FPN |
| OV4689 Bayer 감도 (R/G/B) | 0.916 / 1.000 / 0.854 | QE 비율 |
| 최적 가중치 (R/G/B) | 0.4544 / 0.3345 / 0.2111 | Inverse Variance Weighting |

### 13.4 별 카탈로그

| 항목 | 값 |
|------|-----|
| 출처 | Hipparcos |
| 별 수 | 4,992개 (mag ≤ 6.0) |
| 파일 (CSV) | `data/Hipparcos_Below_6.0.csv` |
| 파일 (K-vector) | `data/star_catalog_kvector.mat` (477 MB) |
| CSV 컬럼 | Index, Star_ID, RA [rad], DEC [rad], Magnitude |

K-vector 인덱스는 DEC 범위 검색을 $O(\log n)$으로 가속한다.

---

*최종 업데이트: 2026-02-23*
