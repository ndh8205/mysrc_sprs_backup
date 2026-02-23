function [gray_img, bayer_img, star_info] = simulate_star_image_realistic(ra_deg, de_deg, roll_deg, sensor_params)
% SIMULATE_STAR_IMAGE_REALISTIC 실제 별 카탈로그 기반 별 이미지 시뮬레이션
%
% 입력:
%   ra_deg   - Right Ascension (degrees)
%   de_deg   - Declination (degrees)
%   roll_deg - Roll angle (degrees)
%   sensor_params - 센서 파라미터 구조체
%
% 출력:
%   gray_img  - Grayscale 이미지 (이상적)
%   bayer_img - Bayer 패턴 이미지 (RGGB)
%   star_info - 별 정보 구조체
%
% 기반: star_simulator_main.m (Brian Catraguna's Python code 변환)
%
% [알고리즘 개요]
%   별 이미지 시뮬레이션 파이프라인:
%     1. 센서 파라미터 설정 (OV4689 기반)
%     2. 입력 자세(RA/DEC/Roll)를 라디안으로 변환
%     3. FOV 계산 → 검색 영역 결정
%     4. 회전행렬 생성 (천구 좌표계 → 센서 좌표계)
%     5. 별 카탈로그 로드 (Hipparcos, 6등급 이하)
%     6. FOV 내 별 검색 (RA/DEC 범위 필터링)
%     7. 천구 좌표 → 센서 좌표 → 이미지 좌표 변환
%     8. 핀홀 모델로 픽셀 좌표 계산
%     9. 등급 기반 플럭스 계산 (Pogson 공식)
%    10. 가우시안 PSF로 별 렌더링
%    11. 노이즈 추가 (다크 전류 + 샷 노이즈 + 읽기 노이즈)
%    12. Bayer 패턴(RGGB) 이미지 생성
%
% [센서 사양] 지원 센서: IMX296(GS) / IMX477(RS,BSI) / IMX219(RS) + OV4689
%   get_sensor_preset() 함수로 프리셋 로드 가능
%   롤링셔터 모델: 행별 시간차에 의한 별 위치 이동 시뮬레이션
%   ADC: 센서별 8/10/12-bit
%
% [좌표계 변환 흐름]
%   천구 좌표 (RA, DEC) [deg]
%     → 3D 단위벡터 (ECI, 지구 중심 관성 좌표계)
%       → 센서 좌표 (Body frame, 회전행렬 적용)
%         → 이미지 좌표 (핀홀 투영) [m]
%           → 픽셀 좌표 (좌상단 원점) [pixel]
%
% [관련 함수]
%   bayer_to_rgb_cfa.m      - Bayer → RGB 디모자이킹 (CFA 보간)
%   rgb_to_gray_fpga.m      - RGB → Grayscale 변환 (FPGA 방식)
%   create_rotation_matrix() - 오일러 회전행렬 생성 (이 파일 내 헬퍼)
%   draw_star_psf()          - 가우시안 PSF 렌더링 (이 파일 내 헬퍼)
%   set_default()            - 구조체 기본값 설정 (이 파일 내 헬퍼)

%% 기본 파라미터
if nargin < 4
    sensor_params = struct();
end

% 센서 기본 파라미터 (get_sensor_preset()으로 오버라이드 가능)
sensor_params = set_default(sensor_params, 'myu', 2e-6);      % 픽셀 크기 [m]
sensor_params = set_default(sensor_params, 'f', 0.01042);     % 초점거리 [m]
sensor_params = set_default(sensor_params, 'l', 1280);        % 가로 픽셀
sensor_params = set_default(sensor_params, 'w', 720);         % 세로 픽셀
sensor_params = set_default(sensor_params, 'mag_limit', 6.5); % 최대 등급
sensor_params = set_default(sensor_params, 'adc_bits', 8);    % ADC 비트 수
sensor_params = set_default(sensor_params, 'full_well', 10000); % Full well [e-]
sensor_params = set_default(sensor_params, 'psf_sigma', 1.2); % PSF sigma [pixel]

% === 롤링셔터 파라미터 ===
% shutter_type: 'global' (전체 동시 노출) | 'rolling' (행별 순차 노출)
% readout_time: 전체 프레임 읽기 시간 [s] (rolling일 때만 의미)
% angular_velocity: 위성 자세 변화 속도 [deg/s] (3축: [wx, wy, wz])
%   롤링셔터 효과: 행 r의 노출 시작 = t0 + (r/H) * readout_time
%   -> 각 행에서 별의 투영 위치가 미세하게 이동
sensor_params = set_default(sensor_params, 'shutter_type', 'global');
sensor_params = set_default(sensor_params, 'readout_time', 0);
sensor_params = set_default(sensor_params, 'angular_velocity', [0, 0, 0]);

% === 노출/게인 파라미터 (실제 OV4689 레지스터 기반) ===
% 노출 시간 (ov4689.c sensor_s_exp 함수 참조)
%   - 기본값 0xc350 (50000) → 약 22ms (60Hz, VTS=2350 기준)
%   - exp_time = (exp_val >> 4) * line_time
%   - line_time = 1/(60Hz * 2350) ≈ 7.09µs
sensor_params = set_default(sensor_params, 'exposure_time', 0.022);   % 노출 시간 [초]

% 아날로그 게인 (ov4689.c sensor_s_gain 함수 참조)
%   - 범위: 1x ~ 64x (gain_val: 16 ~ 1023)
%   - 기본 레지스터값 0x0FFF (최대 게인)
sensor_params = set_default(sensor_params, 'analog_gain', 16.0);      % 아날로그 게인 [배]

% 디지털 게인 (레지스터 0x352a 등)
%   - 기본값 0x08 = 1.0x
sensor_params = set_default(sensor_params, 'digital_gain', 1.0);      % 디지털 게인 [배]

% 양자 효율 (Quantum Efficiency)
%   - OV4689 CMOS 센서 기준 약 50%
sensor_params = set_default(sensor_params, 'quantum_efficiency', 0.5);

% === 환경 파라미터 ===
sensor_params = set_default(sensor_params, 'environment', 'space');    % 'space' | 'ground'
sensor_params = set_default(sensor_params, 'sensor_temp', -20);        % 센서 온도 [deg C] (우주: -20~-40)

% === 노이즈 파라미터 ===
sensor_params = set_default(sensor_params, 'read_noise', 3);           % 읽기 노이즈 [e- RMS]
sensor_params = set_default(sensor_params, 'add_noise', true);

% 다크 전류 온도 모델 (Arrhenius)
%   I_dark(T) = I_ref * 2^((T - T_ref) / T_double)
%   @25 deg C: 0.1 e-/px/s (OV4689 데이터시트 기반)
%   @-20 deg C: 0.1 * 2^(-45/6.5) = 0.00071 e-/px/s (140배 감소)
sensor_params = set_default(sensor_params, 'dark_current_ref', 0.1);   % 기준 다크전류 [e-/px/s] @25 deg C
sensor_params = set_default(sensor_params, 'dark_temp_ref', 25);       % 기준 온도 [deg C]
sensor_params = set_default(sensor_params, 'dark_temp_double', 6.5);   % 2배 증가 온도 [deg C]

% 우주방사선 (Cosmic Ray)
%   LEO 530km: ~5 hits/cm^2/s (SAA 외 평균)
%   OV4689 센서 면적: 0.256cm x 0.144cm = 0.0369 cm^2
%   22ms 노출당: 5 * 0.0369 * 0.022 = 0.004 hits/frame (평균)
sensor_params = set_default(sensor_params, 'cosmic_ray_rate', 5);      % [hits/cm^2/s]
sensor_params = set_default(sensor_params, 'cosmic_ray_energy', 1000); % 평균 에너지 [e-/hit]

% 스트레이라이트 (배경 광)
%   태양/지구/달 반사광 → 균일 배경 증가
%   바플 성능에 따라 0~10 e-/px/s
sensor_params = set_default(sensor_params, 'stray_light', 0);          % [e-/px/s]

% Fixed Pattern Noise (센서 제조 편차)
%   DSNU: 다크 신호 불균일 (가산적, 온도 비례)
%   PRNU: 광응답 불균일 (승산적, 1% = 일반적 CMOS)
sensor_params = set_default(sensor_params, 'dsnu', 0.5);               % DSNU [e-/s RMS]
sensor_params = set_default(sensor_params, 'prnu', 0.01);              % PRNU [비율] (1%)

% === Bayer 채널 감도 (OV4689 실측 QE 비율) ===
%   R: 46.7%, G: 51.1%, B: 43.6% (상대값 = 각 채널 / G 채널)
sensor_params = set_default(sensor_params, 'sensitivity_R', 0.916);    % R/G = 46.7/51.1
sensor_params = set_default(sensor_params, 'sensitivity_G', 1.0);      % 기준
sensor_params = set_default(sensor_params, 'sensitivity_B', 0.854);    % B/G = 43.6/51.1

%% Convert to radians
% --- 각도 → 라디안 변환 ---
% 이후 삼각함수(sin, cos, atan) 계산을 위해 라디안 단위 필요
% MATLAB의 삼각함수는 라디안 입력을 기대함
ra = deg2rad(ra_deg);
de = deg2rad(de_deg);
roll = deg2rad(roll_deg);

%% Calculate FOV
myu = sensor_params.myu;
f = sensor_params.f;
l = sensor_params.l;
w = sensor_params.w;

FOVy = rad2deg(2 * atan((myu*w/2) / f));
FOVx = rad2deg(2 * atan((myu*l/2) / f));

% === FOV 유도 ===
% FOV 공식 유도:
%   Half-angle: tan(theta/2) = (sensor_half_size) / focal_length
%   sensor_half_size = (pixel_count / 2) * pixel_size = (l/2) * myu [m]
%   theta = 2 * atan(sensor_half_size / f) [rad]
%
%   OV4689 + 10.42mm 렌즈 대입:
%     FOVx = 2 * atan((2e-6 * 1280/2) / 0.01042) = 2 * atan(0.1228) = 14.02 [deg]
%     FOVy = 2 * atan((2e-6 * 720/2) / 0.01042)  = 2 * atan(0.0692) = 7.91  [deg]
%     대각 FOV = sqrt(14.02^2 + 7.91^2) = 16.1 [deg]
%
%   도식:
%           <--- FOVx = 14.02 deg --->
%     +----+---------------------------+----+
%     |    |                           |    | ^
%     |    |       센서 평면            |    | | FOVy = 7.91 deg
%     |    |      (1280 x 720)         |    | v
%     +----+---------------------------+----+
%                      |
%                      | f = 10.42 mm
%                      |
%                   [렌즈]

star_info.FOVx = FOVx;
star_info.FOVy = FOVy;
star_info.ra_deg = ra_deg;
star_info.de_deg = de_deg;
star_info.roll_deg = roll_deg;

%% Create Rotation Matrix
% M: 3x3 회전행렬, 천구 좌표계(ECI) → 센서 좌표계(Body) 변환
% M_transpose (= M^-1): 센서 좌표계 → 천구 좌표계 변환
% 자세한 유도 과정은 create_rotation_matrix() 함수 참조
M = create_rotation_matrix(ra, de, roll);

%% Load Star Catalogue (자동 경로 감지: 포터블 / 프로젝트)
script_dir = fileparts(mfilename('fullpath'));
% 포터블: core/ → parent/data/, 프로젝트: bayer/ → 4단계 → Space_SLAM/data/
data_dir = fullfile(fileparts(script_dir), 'data');
if ~exist(data_dir, 'dir')
    project_root = fileparts(fileparts(fileparts(fileparts(script_dir))));
    data_dir = fullfile(project_root, 'data');
end

catalog_path = fullfile(data_dir, 'star_catalog_kvector.mat');
csv_path = fullfile(data_dir, 'Hipparcos_Below_6.0.csv');

if isfield(sensor_params, 'preloaded_catalog') && ~isempty(sensor_params.preloaded_catalog)
    % GUI에서 사전 로드된 카탈로그 사용 (477MB 반복 로드 방지)
    catalog_data = sensor_params.preloaded_catalog;

    ra_stars = catalog_data.star_catalog.RA;
    de_stars = catalog_data.star_catalog.DEC;
    magnitudes = catalog_data.star_catalog.Magnitude;
elseif exist(catalog_path, 'file')
    data = load(catalog_path);
    catalog_data = data.catalog_data;

    % 카탈로그에서 별 정보 추출
    ra_stars = catalog_data.star_catalog.RA;
    de_stars = catalog_data.star_catalog.DEC;
    magnitudes = catalog_data.star_catalog.Magnitude;
elseif exist(csv_path, 'file')
    % 대체: CSV 파일 사용
    opts = detectImportOptions(csv_path);
    opts.VariableNamingRule = 'preserve';
    star_catalogue = readtable(csv_path, opts);
    ra_stars = star_catalogue.RA;
    de_stars = star_catalogue.DE;
    magnitudes = star_catalogue.Magnitude;
else
    error('별 카탈로그를 찾을 수 없습니다. data/ 폴더에 star_catalog_kvector.mat 또는 Hipparcos_Below_6.0.csv 파일이 필요합니다.');
end

%% Search for stars within FOV
R = sqrt(deg2rad(FOVx)^2 + deg2rad(FOVy)^2) / 2;
alpha_start = ra - R/cos(de);
alpha_end = ra + R/cos(de);
delta_start = de - R;
delta_end = de + R;

% --- FOV 검색 영역 설명 ---
% R: FOV 검색 반경 [rad] = 대각 FOV의 절반
%   대각선 = sqrt(FOVx^2 + FOVy^2), 그 절반을 검색 반경으로 사용
%   원형 검색 영역이 직사각형 FOV를 완전히 포함 (안전 마진)
%
%        원형 검색 영역 (반경 R)
%       .---.-----.-----.---.
%      /    |     |     |    \
%     /   +---+-------+---+  \
%    |    | 실제 FOV (직사각형)|
%     \   +---+-------+---+  /
%      \    |     |     |    /
%       '---'-----'-----'---'
%
% cos(de) 보정:
%   적경(RA)은 적도 기준 각도이므로, 적위(DEC)가 극에 가까울수록
%   동일 각거리에 필요한 RA 범위가 넓어짐 (메르카토르 투영과 유사)
%   예: DEC=60 deg에서 1 deg 각거리 → RA 범위 = 1/cos(60) = 2 [deg]
%   이 보정이 없으면 극 부근에서 별을 놓칠 수 있음

% Find stars in range
star_within_ra = (alpha_start <= ra_stars) & (ra_stars <= alpha_end);
star_within_de = (delta_start <= de_stars) & (de_stars <= delta_end);
stars_in_fov = star_within_ra & star_within_de;

% 등급 제한
mag_ok = magnitudes <= sensor_params.mag_limit;
stars_in_fov = stars_in_fov & mag_ok;

% Filter stars
ra_i = ra_stars(stars_in_fov);
de_i = de_stars(stars_in_fov);
mag_i = magnitudes(stars_in_fov);

%% Convert to sensor coordinates
star_sensor_coords = zeros(length(ra_i), 3);
M_transpose = M';

% --- 천구 좌표 → 센서 좌표 변환 ---
% 천구 좌표 → 3D 방향 벡터 (단위 벡터):
%   dir_vector = [cos(RA)*cos(DEC), sin(RA)*cos(DEC), sin(DEC)]^T
%   이 벡터는 길이=1인 단위 벡터, 천구 좌표계(ECI) 기준
%
% M_transpose (= M^T = M^-1) 적용:
%   센서 좌표계에서의 별 방향 벡터 [Xs, Ys, Zs]
%   Zs > 0이면 센서 전방 (카메라가 보는 방향)
%   Zs <= 0이면 센서 후방 (이미지에 나타나지 않음)
%   ※ 이 코드에서는 Z > 0 체크를 생략 (FOV 검색으로 대부분 걸러짐)
%
%   (ECI)                    (Body)
%   [cos(RA)cos(DEC)]        [Xs]
%   [sin(RA)cos(DEC)]  --M^T-->  [Ys]
%   [sin(DEC)        ]        [Zs]

for i = 1:length(ra_i)
    dir_vector = [cos(ra_i(i))*cos(de_i(i));
                  sin(ra_i(i))*cos(de_i(i));
                  sin(de_i(i))];
    star_sensor_coords(i, :) = (M_transpose * dir_vector)';
end

%% Convert to image coordinates (star_simulator 방식 - z 체크 없음)
star_loc = zeros(length(ra_i), 2);
for i = 1:length(ra_i)
    x = f * (star_sensor_coords(i,1) / star_sensor_coords(i,3));
    y = f * (star_sensor_coords(i,2) / star_sensor_coords(i,3));
    star_loc(i, :) = [x, y];
end

% === 핀홀 카메라 투영 모델 ===
% x_img = f * (Xs / Zs) [m]  (센서 평면 위 물리적 위치)
% y_img = f * (Ys / Zs) [m]
% 여기서 f = 초점거리 [m], (Xs, Ys, Zs) = 센서 좌표계 방향 벡터
%
% 원리: 유사삼각형 (similar triangles)
%
%   별 (무한 원점)
%    \
%     \  Xs/Zs
%      \
%   ----+----------+---- 센서 평면
%       |  x_img   |
%       |<-------->|
%       |    f     |
%       +----------+
%      렌즈 (핀홀)
%
%   Xs/Zs = x_img/f  →  x_img = f * Xs/Zs

% pixel_per_length: 물리 좌표 [m] → 픽셀 좌표 변환 계수 [pixel/m]
%   = 1 / myu = 1 / 2e-6 = 500,000 [pixel/m]
%   예: x_img = 0.001 m → 0.001 * 500000 = 500 [pixel]
pixel_per_length = 1 / myu;

%% Convert to pixel coordinates
% --- 이미지 좌표 변환 ---
% 물리 좌표 (센서 중심 = 원점) → 픽셀 좌표 (좌상단 = 원점) 변환
%
%   좌표계 비교:
%     물리 좌표: 원점=센서 중앙, X=오른쪽(+), Y=위쪽(+)
%     이미지 좌표: 원점=좌상단, X=오른쪽(+), Y=아래쪽(+) (MATLAB 기준)
%
%     물리 좌표계:               이미지 좌표계 (MATLAB):
%        Y(+)                    (0,0)+-------> X(+)
%         ^                           |
%         |                           |
%    -----+-----> X(+)                v Y(+)
%         |
%
%   변환 공식:
%     true_x = l/2 + x1pixel  → 중심 원점을 좌상단 원점으로 이동 [pixel]
%     true_y = w/2 - y1pixel  → Y축 반전 (이미지: 아래가 +, 물리: 위가 +) [pixel]
pixel_coords = [];
filtered_mag = [];
true_centroids = [];

for i = 1:size(star_loc, 1)
    x1 = star_loc(i, 1);
    y1 = star_loc(i, 2);

    x1pixel = pixel_per_length * x1;
    y1pixel = pixel_per_length * y1;

    % Check if within bounds
    if abs(x1pixel) > l/2 || abs(y1pixel) > w/2
        continue;
    end

    pixel_coords = [pixel_coords; x1pixel, y1pixel];
    filtered_mag = [filtered_mag; mag_i(i)];

    % 실제 이미지 좌표 (centroid)
    true_x = l/2 + x1pixel;
    true_y = w/2 - y1pixel;
    true_centroids = [true_centroids; true_x, true_y];
end

star_info.num_stars = length(filtered_mag);
star_info.pixel_coords = pixel_coords;
star_info.magnitudes = filtered_mag;
star_info.true_centroids = true_centroids;

%% Create Grayscale image (ideal)
gray_img = zeros(w, l);

% 센서 파라미터 추출
exposure_time = sensor_params.exposure_time;       % [초]
analog_gain = sensor_params.analog_gain;           % [배]
digital_gain = sensor_params.digital_gain;         % [배]
quantum_eff = sensor_params.quantum_efficiency;    % [0~1]
adc_max = 2^sensor_params.adc_bits - 1;            % ADC 최대값

% 플럭스 모델 파라미터 (루프 밖에서 1회 계산)
ref_mag = 6.0;
ref_photon_flux_ground = 96;
if strcmp(sensor_params.environment, 'space')
    ref_photon_flux = ref_photon_flux_ground / 0.56;  % ~171 photons/s
else
    ref_photon_flux = ref_photon_flux_ground;
end
sigma = sensor_params.psf_sigma;

% === 모션블러 + 롤링셔터 사전 계산 ===
%
% 두 가지 별도 효과:
%   1. 모션블러 (exposure blur): 노출 시간 동안 별이 이동하며 1자로 늘어짐
%      - GS/RS 공통, angular_velocity > 0이면 발생
%      - 별의 PSF가 점 → 선분(line) 으로 변환
%      - blur_length [px] = omega_rad * (f/myu) * exposure_time
%
%   2. 롤링셔터 (row shift): 각 행의 노출 시작 시간이 다름
%      - RS 전용, 별의 중심 위치가 행마다 다름
%      - 효과 크기: 위성 일반 각속도(0.5~5 deg/s)에서 PSF 내 이동 << sigma
%        → 별은 여전히 점/선으로 보이되, 전체 위치만 행에 따라 이동
%
omega_deg = sensor_params.angular_velocity;  % [deg/s] (3-axis)
omega_rad = deg2rad(omega_deg);              % [rad/s]
has_motion = any(omega_rad ~= 0);

% 모션블러: 노출 시간 동안의 별 이동량 [pixel]
if has_motion
    blur_dx = omega_rad(2) * (f / myu) * exposure_time;   % yaw → x
    blur_dy = -omega_rad(1) * (f / myu) * exposure_time;  % pitch → y
    blur_length = sqrt(blur_dx^2 + blur_dy^2);
else
    blur_dx = 0; blur_dy = 0; blur_length = 0;
end
star_info.motion_blur_px = blur_length;

% 롤링셔터: 행별 시간 오프셋에 의한 추가 이동
is_rolling = strcmp(sensor_params.shutter_type, 'rolling') && ...
             sensor_params.readout_time > 0;

if is_rolling
    % 각 행의 노출 시작 시간 오프셋 (행 1 = t=0, 행 H = t=readout_time)
    row_dt = (0:w-1)' / w * sensor_params.readout_time;  % [w x 1]
    rs_dx_per_row = omega_rad(2) * (f / myu) * row_dt;   % [w x 1]
    rs_dy_per_row = -omega_rad(1) * (f / myu) * row_dt;  % [w x 1]
    star_info.rolling_shutter = true;
    star_info.max_rs_shift_px = max(abs([rs_dx_per_row; rs_dy_per_row]));
else
    star_info.rolling_shutter = false;
    star_info.max_rs_shift_px = 0;
end

% 모션블러 샘플 수 결정 (blur_length 기반, 서브픽셀 정밀도)
if blur_length > 0.1
    n_blur_samples = max(3, round(blur_length * 2));  % 최소 3점, blur 1px당 2점
    % 노출 시간을 n_blur_samples 등분하여 PSF를 누적
    blur_offsets_x = linspace(-blur_dx/2, blur_dx/2, n_blur_samples);
    blur_offsets_y = linspace(-blur_dy/2, blur_dy/2, n_blur_samples);
else
    n_blur_samples = 1;
    blur_offsets_x = 0;
    blur_offsets_y = 0;
end

for i = 1:length(filtered_mag)
    x = l/2 + pixel_coords(i,1);
    y = w/2 - pixel_coords(i,2);
    mag = filtered_mag(i);

    % 광자 플럭스 (Pogson)
    photon_flux = ref_photon_flux * 10^(-0.4 * (mag - ref_mag));
    electrons = photon_flux * exposure_time * quantum_eff;
    total_flux = electrons * analog_gain * digital_gain;

    if is_rolling && has_motion
        % === 롤링셔터 + 모션블러 ===
        % 각 행에서: (1) RS에 의한 위치 이동 + (2) 모션블러에 의한 선분 PSF
        gray_img = draw_star_psf_rolling_blur(gray_img, x, y, sigma, total_flux, ...
            rs_dx_per_row, rs_dy_per_row, blur_offsets_x, blur_offsets_y, n_blur_samples);
    elseif has_motion
        % === 글로벌 셔터 + 모션블러 ===
        % 위치 이동 없이, 모션블러만 적용 (선분 PSF)
        flux_per_sample = total_flux / n_blur_samples;
        for bi = 1:n_blur_samples
            gray_img = draw_star_psf(gray_img, ...
                x + blur_offsets_x(bi), y + blur_offsets_y(bi), ...
                sigma, flux_per_sample);
        end
    else
        % === 글로벌 셔터, 정지 ===
        gray_img = draw_star_psf(gray_img, x, y, sigma, total_flux);
    end
end

star_info.ideal_gray = gray_img;

%% Add noise to grayscale (optional)
total_gain = analog_gain * digital_gain;

% === 온도 기반 다크 전류 계산 (Arrhenius 모델) ===
%   I_dark(T) = I_ref * 2^((T - T_ref) / T_double)
%   -20 deg C: 0.1 * 2^(-45/6.5) = 0.00071 e-/px/s (140x 감소)
%   +25 deg C: 0.1 e-/px/s (기준)
T_sensor = sensor_params.sensor_temp;
dark_rate_actual = sensor_params.dark_current_ref ...
    * 2^((T_sensor - sensor_params.dark_temp_ref) / sensor_params.dark_temp_double);
dark_electrons = dark_rate_actual * exposure_time;

% === 스트레이라이트 배경 (균일) ===
stray_electrons = sensor_params.stray_light * exposure_time;

% === Fixed Pattern Noise 맵 생성 (프레임 불변) ===
% DSNU: 다크 신호 불균일 (가산적, 온도 비례)
if sensor_params.dsnu > 0
    rng_state = rng;
    rng(42);  % 고정 시드 -> 매 프레임 동일한 패턴
    dsnu_map = sensor_params.dsnu * randn(w, l) ...
        * (dark_rate_actual / sensor_params.dark_current_ref);
    dsnu_electrons = dsnu_map * exposure_time;
    rng(rng_state);
else
    dsnu_electrons = 0;
end

% PRNU: 광응답 불균일 (승산적, 1% RMS)
if sensor_params.prnu > 0
    rng_state = rng;
    rng(43);
    prnu_map = 1 + sensor_params.prnu * randn(w, l);
    rng(rng_state);
else
    prnu_map = 1;
end

if sensor_params.add_noise
    % === 노이즈 모델 (우주 환경 대응) ===
    %
    % 물리 체인:
    %   광자 -> 전자 (Poisson) -> FPN -> cosmic ray -> 게인 -> 읽기 노이즈
    %
    % 1. ADU -> 전자 역변환 (게인 제거) + PRNU 적용
    signal_electrons = (gray_img / total_gain) .* prnu_map;

    % 2. 배경 전자 합산 (다크 + 스트레이라이트 + DSNU)
    total_electrons = signal_electrons + dark_electrons + stray_electrons + dsnu_electrons;

    % 3. 샷 노이즈 (Poisson) - 전자 단위
    noisy_electrons = poissrnd(max(0, total_electrons));

    % 4. 우주방사선 (Cosmic Ray Hits)
    if sensor_params.cosmic_ray_rate > 0
        % 센서 면적 [cm^2]: (pixel_size * width) * (pixel_size * height)
        sensor_area_cm2 = (sensor_params.myu * l * 100) * (sensor_params.myu * w * 100);
        expected_hits = sensor_params.cosmic_ray_rate * sensor_area_cm2 * exposure_time;
        n_hits = poissrnd(expected_hits);

        for h = 1:n_hits
            hit_row = randi(w);
            hit_col = randi(l);
            hit_energy = sensor_params.cosmic_ray_energy * (0.5 + rand());
            % 3x3 클러스터 (중심 집중 가우시안)
            for dr = -1:1
                for dc = -1:1
                    r = hit_row + dr;
                    c = hit_col + dc;
                    if r >= 1 && r <= w && c >= 1 && c <= l
                        frac = exp(-(dr^2 + dc^2) / 0.8);
                        noisy_electrons(r, c) = noisy_electrons(r, c) + hit_energy * frac;
                    end
                end
            end
        end
    end

    % 5. 전자 -> ADU (게인 재적용)
    gray_noisy = noisy_electrons * total_gain;

    % 6. 읽기 노이즈 (Gaussian) - ADU 단위
    read_noise_adu = sensor_params.read_noise * total_gain;
    gray_noisy = gray_noisy + read_noise_adu * randn(size(gray_noisy));

    % 7. ADC 클램핑 (adc_bits에 따라)
    gray_noisy = max(0, min(adc_max, gray_noisy));
    gray_img = uint16(round(gray_noisy));
else
    % 노이즈 없이 클램핑만
    gray_img = uint16(round(max(0, min(adc_max, gray_img))));
end

%% Create Bayer pattern image (RGGB)
bayer_img = zeros(w, l);

% 먼저 노이즈 없는 ideal 이미지로 Bayer 생성
ideal_gray = star_info.ideal_gray;

% === Bayer 채널 감도 적용 ===
% RGGB 패턴에서 각 색상 필터의 상대 감도를 적용합니다.
%
% RGGB 2x2 타일 매핑 (짝수/홀수 행-열 기준):
%   r_idx = mod(row-1, 2), c_idx = mod(col-1, 2)
%
%   r_idx=0, c_idx=0 → R  (빨간색, 감도 1.0)
%   r_idx=0, c_idx=1 → Gr (녹색1, 감도 1.0)
%   r_idx=1, c_idx=0 → Gb (녹색2, 감도 1.0)
%   r_idx=1, c_idx=1 → B  (파란색, 감도 0.9)
%
%   +-------+-------+-------+-------+
%   | R 1.0 |Gr 1.0 | R 1.0 |Gr 1.0 |  <- 짝수 행 (r_idx=0)
%   +-------+-------+-------+-------+
%   |Gb 1.0 | B 0.9 |Gb 1.0 | B 0.9 |  <- 홀수 행 (r_idx=1)
%   +-------+-------+-------+-------+
%   | R 1.0 |Gr 1.0 | R 1.0 |Gr 1.0 |
%   +-------+-------+-------+-------+
%   |Gb 1.0 | B 0.9 |Gb 1.0 | B 0.9 |
%   +-------+-------+-------+-------+
%     짝수열   홀수열   짝수열   홀수열
%    (c_idx=0)(c_idx=1)
%
% B=0.9인 이유:
%   OV4689 센서의 Blue 채널은 Red/Green 대비 약 10% 낮은 감도
%   (센서 데이터시트 기반 근사값)
%   별빛은 주로 가시광~적외선이므로 Blue 감도가 약간 낮은 것이 현실적
for row = 1:w
    for col = 1:l
        r_idx = mod(row-1, 2);
        c_idx = mod(col-1, 2);

        if r_idx == 0 && c_idx == 0
            sensitivity = sensor_params.sensitivity_R;  % R
        elseif r_idx == 1 && c_idx == 1
            sensitivity = sensor_params.sensitivity_B;  % B
        else
            sensitivity = sensor_params.sensitivity_G;  % G
        end

        bayer_img(row, col) = ideal_gray(row, col) * sensitivity;
    end
end

% Add noise (확장 노이즈 모델 적용)
if sensor_params.add_noise
    % 1. ADU -> 전자 (게인 제거) + PRNU
    bayer_electrons = (bayer_img / total_gain) .* prnu_map;

    % 2. 배경 합산 (다크 + 스트레이라이트 + DSNU)
    bayer_electrons = bayer_electrons + dark_electrons + stray_electrons + dsnu_electrons;

    % 3. 샷 노이즈 (Poisson)
    bayer_electrons = poissrnd(max(0, bayer_electrons));

    % 4. 우주방사선 (Cosmic Ray) - Bayer용 별도 히트
    if sensor_params.cosmic_ray_rate > 0
        sensor_area_cm2 = (sensor_params.myu * l * 100) * (sensor_params.myu * w * 100);
        expected_hits = sensor_params.cosmic_ray_rate * sensor_area_cm2 * exposure_time;
        n_hits_bayer = poissrnd(expected_hits);
        for h = 1:n_hits_bayer
            hit_row = randi(w);
            hit_col = randi(l);
            hit_energy = sensor_params.cosmic_ray_energy * (0.5 + rand());
            for dr = -1:1
                for dc = -1:1
                    r = hit_row + dr;
                    c = hit_col + dc;
                    if r >= 1 && r <= w && c >= 1 && c <= l
                        frac = exp(-(dr^2 + dc^2) / 0.8);
                        bayer_electrons(r, c) = bayer_electrons(r, c) + hit_energy * frac;
                    end
                end
            end
        end
    end

    % 5. 전자 -> ADU (게인 재적용)
    bayer_img = bayer_electrons * total_gain;

    % 6. 읽기 노이즈 (Gaussian)
    bayer_img = bayer_img + read_noise_adu * randn(size(bayer_img));
end

% ADC 클램핑
bayer_img = max(0, min(adc_max, bayer_img));
bayer_img = uint16(round(bayer_img));

star_info.sensor_params = sensor_params;

end

%% Helper Functions

% --- set_default: 구조체 필드 기본값 설정 유틸리티 ---
% 필드가 아직 없으면 기본값으로 설정, 이미 있으면 유지
function params = set_default(params, field, value)
    if ~isfield(params, field)
        params.(field) = value;
    end
end

function M = create_rotation_matrix(ra, de, roll)
    % CREATE_ROTATION_MATRIX 적경/적위/롤 → 3D 회전행렬 생성
    %
    % [설명]
    %   천구 좌표계(ECI: Earth-Centered Inertial)에서
    %   센서 좌표계(Body frame)로의 변환 행렬을 생성합니다.
    %
    %   3단계 오일러 회전 (Z-X-Z 순서):
    %     M1: Z축 회전 (적경 RA 기반) → 적경 방향으로 회전
    %     M2: X축 회전 (적위 DEC 기반) → 적위 방향으로 기울임
    %     M3: Z축 회전 (롤 Roll) → 광축 중심 회전
    %
    %   pi/2 오프셋 설명:
    %     ra_exp = ra - pi/2:
    %       천문학 관례에서 RA=0는 춘분점 방향 (Vernal Equinox)
    %       센서 좌표계의 기준 방향과 90 deg 차이 보정
    %     de_exp = de + pi/2:
    %       DEC=0 (적도)에서 센서 광축이 적도면에 수직이 되도록 보정
    %       DEC = 0 deg → de_exp = 90 deg → 적도면을 향함
    %
    %   변환 도식:
    %     천구(ECI)  --M1(RA)--> --M2(DEC)--> --M3(Roll)--> 센서(Body)
    %
    %   결과: M = M1 * M2 * M3 (행렬 곱 순서 = 변환 적용 역순)
    %
    % [입력]
    %   ra   - 적경 (Right Ascension) [rad]
    %   de   - 적위 (Declination) [rad]
    %   roll - 롤 각도 (Roll) [rad]
    %
    % [출력]
    %   M    - 3x3 회전행렬 (직교행렬: M * M^T = I)
    %          천구 → 센서 좌표 변환에는 M^T (= M^-1) 사용

    % --- 오프셋 적용 ---
    % RA에서 pi/2를 빼고, DEC에 pi/2를 더하여 좌표계 정렬
    ra_exp = ra - (pi/2);
    de_exp = de + (pi/2);

    % M1: Z축 회전 (적경 보정) [rad]
    %   cos/sin(ra - pi/2) 적용
    M1 = [cos(ra_exp), -sin(ra_exp), 0;
          sin(ra_exp),  cos(ra_exp), 0;
          0,            0,           1];

    % M2: X축 회전 (적위 보정) [rad]
    %   cos/sin(de + pi/2) 적용
    M2 = [1,  0,            0;
          0,  cos(de_exp), -sin(de_exp);
          0,  sin(de_exp),  cos(de_exp)];

    % M3: Z축 회전 (롤) [rad]
    M3 = [cos(roll), -sin(roll), 0;
          sin(roll),  cos(roll), 0;
          0,          0,         1];

    % 최종 회전행렬: M1 * M2 * M3
    M = M1 * M2 * M3;
end

function img = draw_star_psf(img, cx, cy, sigma, total_flux)
    % DRAW_STAR_PSF 가우시안 PSF로 별을 이미지에 렌더링
    %
    % [설명]
    %   2D 가우시안 함수를 이미지에 누적(additive) 방식으로 렌더링합니다.
    %   에너지 보존 (Energy Conserving): PSF 전체 적분값 = total_flux [ADU]
    %
    % [입력]
    %   img        - 대상 이미지 행렬 [ADU]
    %   cx, cy     - 별 중심 좌표 (서브픽셀 정밀도) [pixel]
    %   sigma      - PSF 표준편차 [pixel]
    %   total_flux - PSF 적분값 (별의 총 신호량) [ADU]
    %
    % [출력]
    %   img        - 별이 추가된 이미지 [ADU]
    %
    % [정규화]
    %   2D Gaussian: I(x,y) = A * exp(-(x^2+y^2)/(2*sigma^2))
    %   적분: integral(I dx dy) = 2*pi*sigma^2*A = total_flux
    %   따라서: A = total_flux / (2*pi*sigma^2) [ADU/pixel]
    %
    % [윈도우]
    %   +-6*sigma 범위로 렌더링 (99.99% 에너지 포함)
    %   sigma=1.2 pixel → 윈도우 = +-8 pixel → 15x15 pixel 영역

    % 물리적으로 정확한 PSF 렌더링
    % total_flux: PSF 적분값 (총 광자 수, ADU)
    % sigma: PSF 표준편차 (픽셀)
    %
    % 2D Gaussian: I(x,y) = A * exp(-(x^2+y^2)/(2*sigma^2))
    % 적분값: integral = 2 * pi * sigma^2 * A
    % 따라서: A = total_flux / (2 * pi * sigma^2)

    [h, w] = size(img);

    % 피크 값 계산 (정규화)
    peak_amplitude = total_flux / (2 * pi * sigma^2);

    % 윈도우 크기 (6-sigma 범위)
    win = ceil(6 * sigma);

    % 렌더링 범위를 이미지 경계 내로 클램핑 [pixel]
    x_start = max(1, round(cx) - win);
    x_end = min(w, round(cx) + win);
    y_start = max(1, round(cy) - win);
    y_end = min(h, round(cy) + win);

    for y = y_start:y_end
        for x = x_start:x_end
            % 서브픽셀 거리
            dx = x - cx;
            dy = y - cy;
            dist_sq = dx^2 + dy^2;

            % Gaussian PSF
            val = peak_amplitude * exp(-dist_sq / (2 * sigma^2));

            % 누적 (겹치는 별 처리)
            img(y, x) = img(y, x) + val;
        end
    end
end

function img = draw_star_psf_rolling_blur(img, cx, cy, sigma, total_flux, ...
        rs_dx, rs_dy, blur_ox, blur_oy, n_samples)
    % DRAW_STAR_PSF_ROLLING_BLUR 롤링셔터 + 모션블러 PSF 렌더링
    %
    % 두 효과 결합:
    %   1. 롤링셔터: 행 r에서 별 중심이 (cx+rs_dx(r), cy+rs_dy(r))로 이동
    %   2. 모션블러: 각 행의 노출 시간 동안 별이 blur_ox/oy 만큼 이동
    %      → PSF를 노출 궤적(선분)을 따라 n_samples점으로 분할하여 누적
    %
    % 입력:
    %   img        - 대상 이미지 [ADU]
    %   cx, cy     - 별 기준 중심 좌표 (행 1 기준) [pixel]
    %   sigma      - PSF 표준편차 [pixel]
    %   total_flux - PSF 적분값 [ADU]
    %   rs_dx/dy   - 롤링셔터 행별 이동량 [w x 1] [pixel]
    %   blur_ox/oy - 모션블러 샘플 오프셋 [1 x n_samples] [pixel]
    %   n_samples  - 모션블러 샘플 수

    [h, w] = size(img);
    peak_amplitude = total_flux / (2 * pi * sigma^2) / n_samples;
    win = ceil(6 * sigma);

    % 전체 이동 범위 + 모션블러 범위
    max_rs_x = max(abs(rs_dx));
    max_rs_y = max(abs(rs_dy));
    max_blur_x = max(abs(blur_ox));
    max_blur_y = max(abs(blur_oy));

    y_start = max(1, round(cy - max_rs_y - max_blur_y) - win);
    y_end = min(h, round(cy + max_rs_y + max_blur_y) + win);
    x_start = max(1, round(cx - max_rs_x - max_blur_x) - win);
    x_end = min(w, round(cx + max_rs_x + max_blur_x) + win);

    for y = y_start:y_end
        % 이 행에서의 별 중심 (롤링셔터 이동)
        cx_row = cx + rs_dx(y);
        cy_row = cy + rs_dy(y);

        for x = x_start:x_end
            val_total = 0;
            % 모션블러: 노출 궤적을 따라 PSF 누적
            for bi = 1:n_samples
                dx = x - (cx_row + blur_ox(bi));
                dy = y - (cy_row + blur_oy(bi));
                dist_sq = dx^2 + dy^2;
                val_total = val_total + exp(-dist_sq / (2 * sigma^2));
            end
            val = peak_amplitude * val_total;
            if val > 1e-6
                img(y, x) = img(y, x) + val;
            end
        end
    end
end
