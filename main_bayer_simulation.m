%% main_simulation.m
% Star Tracker 별 이미지 시뮬레이션
%
% 목적:
%   Hipparcos 카탈로그 기반 물리적으로 정확한 별 이미지 생성
%   - 실제 별 위치/등급 사용
%   - Pogson 공식 기반 플럭스 계산
%   - 광학 시스템 PSF 적용
%   - 센서 노이즈 모델링 (샷 노이즈, 읽기 노이즈, 다크 전류)
%
% 사용법:
%   1. MATLAB에서 이 파일 실행
%   2. '설정' 섹션에서 관측 방향 변경 가능
%
% 의존성:
%   - 별 카탈로그 파일 필요 (경로 설정 필요)
%   - Image Processing Toolbox
%   - Statistics Toolbox (poissrnd)
%
% 관련 서브 시뮬레이션:
%   - sub_main_1_bayer_comparison.m : Bayer → Gray 변환 방식 비교
%
% 저자: GAiMSat-1 Star Tracker 프로젝트
% 날짜: 2026-01-30

clear; close all; clc;

%% 경로 설정 (포터블 / 프로젝트 자동 감지)
script_dir = fileparts(mfilename('fullpath'));
core_dir = fullfile(script_dir, 'core');
if exist(core_dir, 'dir')
    addpath(core_dir);  % 포터블 모드
else
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model', 'sensor', 'star_tracker', 'bayer'));
end

fprintf('================================================\n');
fprintf('  Star Tracker 별 이미지 시뮬레이션\n');
fprintf('================================================\n\n');

%% ========== 센서 설정 ==========
% OV4689 + 10.42mm 렌즈 기반
sensor_params = struct();
sensor_params.myu = 2e-6;       % 픽셀 크기 [m]
sensor_params.f = 0.01042;      % 초점거리 [m]
sensor_params.l = 1280;         % 가로 픽셀
sensor_params.w = 720;          % 세로 픽셀
sensor_params.mag_limit = 6.5;  % 최대 등급 (육안 가시 한계)

%% ========== 노출/게인 설정 (OV4689 레지스터 기반) ==========
% 노출 시간 (ov4689.c sensor_s_exp 함수 참조)
%   - 레지스터: 0x3500-0x3502
%   - 기본값 0xc350 (50000) → 약 22ms (60Hz 기준)
%   - 범위: 0 ~ 0xFFFFF (약 0 ~ 1초)
sensor_params.exposure_time = 0.022;     % 노출 시간 [초]

% 아날로그 게인 (ov4689.c sensor_s_gain 함수 참조)
%   - 레지스터: 0x3508-0x3509
%   - 범위: 1x ~ 64x
%   - 기본값: 0x0FFF (최대 게인, 약 64x)
sensor_params.analog_gain = 16.0;        % 아날로그 게인 [배]

% 디지털 게인 (레지스터 0x352a)
%   - 기본값: 0x08 = 1.0x
sensor_params.digital_gain = 1.0;        % 디지털 게인 [배]

% 양자 효율 (Quantum Efficiency)
sensor_params.quantum_efficiency = 0.5;  % QE [0~1]

%% ========== 환경/노이즈 설정 ==========
sensor_params.environment = 'space';     % 'space' | 'ground'
sensor_params.sensor_temp = -20;         % 센서 온도 [deg C] (우주: -20~-40, 지상: 25)
sensor_params.read_noise = 3;            % 읽기 노이즈 [e- RMS]
sensor_params.add_noise = true;          % 노이즈 활성화

% 다크전류 Arrhenius 온도 모델
sensor_params.dark_current_ref = 0.1;    % 기준 다크전류 [e-/px/s] @25 deg C
sensor_params.dark_temp_ref = 25;        % 기준 온도 [deg C]
sensor_params.dark_temp_double = 6.5;    % 2배 증가 온도 [deg C]

% 우주 환경 노이즈
sensor_params.cosmic_ray_rate = 5;       % [hits/cm^2/s] (LEO 530km)
sensor_params.cosmic_ray_energy = 1000;  % 평균 에너지 [e-/hit]
sensor_params.stray_light = 0;           % 배경 [e-/px/s] (바플 성능 의존)
sensor_params.dsnu = 0.5;               % DSNU [e-/s RMS]
sensor_params.prnu = 0.01;              % PRNU [비율] (1%)

%% ========== 관측 방향 설정 ==========
% 예시: 오리온 벨트 (삼태성)
% 오리온 벨트 (삼태성) 선택 이유:
%   - 밝은 별 3개 (알니타크 1.7등, 알닐람 1.7등, 민타카 2.2등)가 일렬 배치
%   - 주변에 중간 등급 별도 다수 존재 → 별센서 테스트에 적합
%   - FOV 14°×8° 안에 검출 가능한 별이 충분히 포함됨
ra_deg = 84;      % Right Ascension [deg] (적경)
de_deg = -1;      % Declination [deg] (적위)
roll_deg = 0;     % Roll [deg] (회전)

% 다른 관측 대상 예시:
% - 북극성 부근: ra_deg = 37.95, de_deg = 89.26
% - 큰곰자리 북두칠성: ra_deg = 165, de_deg = 55
% - 전갈자리 안타레스: ra_deg = 247.35, de_deg = -26.43

%% ========== 출력 폴더 ==========
output_dir = fullfile(script_dir, 'output');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% ========== FOV 계산 ==========
% --- FOV (Field of View) 계산 ---
% 핀홀 카메라 모델에서의 시야각 공식:
%   FOV = 2 × atan(센서 반폭 / 초점거리) [rad]
%   센서 반폭 = (픽셀 수 / 2) × 픽셀 크기 [m]
%
% 수치 대입 (OV4689 + 10.42mm 렌즈):
%   가로: FOVx = 2 × atan((2e-6 × 1280/2) / 0.01042)
%        = 2 × atan(0.1228) = 2 × 7.01° = 14.02° [deg]
%   세로: FOVy = 2 × atan((2e-6 × 720/2) / 0.01042)
%        = 2 × atan(0.0692) = 2 × 3.96° = 7.91° [deg]
%   대각: sqrt(14.02² + 7.91²) ≈ 16.1° [deg]
%
% 참고: FOV가 좁을수록 별이 적지만 각 별의 위치 정밀도가 높아짐
%       14° FOV는 별센서에 적합한 범위 (보통 10°~20°)
FOVx = rad2deg(2 * atan((sensor_params.myu*sensor_params.l/2) / sensor_params.f));
FOVy = rad2deg(2 * atan((sensor_params.myu*sensor_params.w/2) / sensor_params.f));

fprintf('센서 파라미터 (OV4689):\n');
fprintf('  해상도: %d x %d 픽셀\n', sensor_params.l, sensor_params.w);
fprintf('  픽셀 크기: %.1f um\n', sensor_params.myu * 1e6);
fprintf('  초점거리: %.2f mm\n', sensor_params.f * 1000);
fprintf('  FOV: %.2f x %.2f deg\n', FOVx, FOVy);
fprintf('  노출 시간: %.1f ms\n', sensor_params.exposure_time * 1000);
fprintf('  아날로그 게인: %.1fx\n', sensor_params.analog_gain);
fprintf('  디지털 게인: %.1fx\n', sensor_params.digital_gain);
fprintf('  양자 효율: %.0f%%\n', sensor_params.quantum_efficiency * 100);
fprintf('  환경: %s\n', sensor_params.environment);
fprintf('  센서 온도: %d deg C\n\n', sensor_params.sensor_temp);

fprintf('관측 방향:\n');
fprintf('  RA: %.2f° (적경)\n', ra_deg);
fprintf('  DEC: %.2f° (적위)\n', de_deg);
fprintf('  Roll: %.2f°\n\n', roll_deg);

%% ========== 1. 별 이미지 생성 ==========
fprintf('1. 별 이미지 생성 (Hipparcos 카탈로그 기반)...\n');
tic;
[gray_ideal, bayer_img, star_info] = simulate_star_image_realistic(ra_deg, de_deg, roll_deg, sensor_params);
gen_time = toc;

fprintf('   생성 시간: %.2f 초\n', gen_time);
fprintf('   FOV 내 별: %d개\n', star_info.num_stars);

if star_info.num_stars > 0
    fprintf('   등급 범위: %.2f ~ %.2f\n', min(star_info.magnitudes), max(star_info.magnitudes));
end
fprintf('\n');

if star_info.num_stars < 3
    warning('별이 너무 적습니다! 다른 RA/DEC를 시도해보세요.');
end

%% ========== 2. 물리 모델 정보 출력 ==========
fprintf('2. 물리 모델:\n');
fprintf('   PSF (Point Spread Function):\n');
fprintf('     - 모델: 2D Gaussian\n');
fprintf('     - sigma = 1.2 픽셀 (광학 시스템에 의해 결정)\n');
fprintf('     - 6-sigma 윈도우 사용\n\n');

fprintf('   신호 모델:\n');
fprintf('     - photon_flux = ref_flux * 10^(-0.4 * (mag - ref_mag))  [Pogson]\n');
fprintf('     - electrons = photon_flux * exposure_time * QE\n');
fprintf('     - ADU = electrons * analog_gain * digital_gain\n');
fprintf('     - 기준: 6등급 = 1000 photons/s (정규화)\n\n');

% 다크전류 온도 보정 계산
dark_rate = sensor_params.dark_current_ref ...
    * 2^((sensor_params.sensor_temp - sensor_params.dark_temp_ref) / sensor_params.dark_temp_double);
total_gain = sensor_params.analog_gain * sensor_params.digital_gain;

fprintf('   노이즈 모델 (%s 환경):\n', sensor_params.environment);
fprintf('     - 샷 노이즈: Poisson(signal + dark + stray)\n');
fprintf('     - 읽기 노이즈: %.1f e- x gain = %.1f ADU RMS\n', ...
    sensor_params.read_noise, sensor_params.read_noise * total_gain);
fprintf('     - 다크 전류: %.2e e-/px/s @%d deg C (기준 %.2f @25 deg C)\n', ...
    dark_rate, sensor_params.sensor_temp, sensor_params.dark_current_ref);
fprintf('       -> %.2e e-/px/frame (%.1f ms)\n', ...
    dark_rate * sensor_params.exposure_time, sensor_params.exposure_time * 1000);
fprintf('     - 스트레이라이트: %.1f e-/px/s\n', sensor_params.stray_light);
fprintf('     - DSNU: %.2f e-/s RMS, PRNU: %.1f%%\n', sensor_params.dsnu, sensor_params.prnu * 100);
fprintf('     - 우주방사선: %.0f hits/cm^2/s\n\n', sensor_params.cosmic_ray_rate);

%% ========== 3. 시각화 ==========
fprintf('3. 시각화...\n');

% Figure 1: 실제 센서 출력 (1:1 픽셀, 실제 크기)
% 이미지 크기에 맞게 Figure 크기 설정 (1280x720 + 여백)
% --- 1:1 픽셀 매핑 Figure ---
% Figure 크기를 이미지 해상도(1280×720)에 정확히 맞춤
% 이렇게 하면 이미지 1픽셀 = 모니터 1픽셀 (확대/축소 없음)
% 별센서 이미지를 실제 크기로 관찰하기 위한 설정
% MenuBar, ToolBar 제거: 순수 이미지 표시만 (디스플레이 공간 확보)
% 'k' (검정) 배경: 우주 배경과 동일하여 시각적으로 자연스러움
% InitialMagnification=100: 1:1 배율로 표시
img_w = sensor_params.l;  % 1280
img_h = sensor_params.w;  % 720
fig1 = figure('Name', sprintf('Star Tracker 센서 출력 (%dx%d)', img_w, img_h), ...
    'Color', 'k', 'MenuBar', 'none', 'ToolBar', 'none');

% Figure 크기를 이미지 크기에 정확히 맞춤
set(fig1, 'Units', 'pixels');
fig1.Position = [50 100 img_w img_h];

% Axes를 Figure 전체에 맞춤 (여백 없음)
ax1 = axes('Parent', fig1, 'Units', 'normalized', 'Position', [0 0 1 1]);

% 실제 센서 출력 이미지 표시 (Bayer + 노이즈)
imshow(bayer_img, 'Parent', ax1, 'InitialMagnification', 100);
axis(ax1, 'off');

% Figure 2: 이상적 Grayscale (1:1 픽셀)
fig2 = figure('Name', sprintf('이상적 Grayscale (%dx%d)', img_w, img_h), ...
    'Color', 'k', 'MenuBar', 'none', 'ToolBar', 'none');
set(fig2, 'Units', 'pixels');
fig2.Position = [100 50 img_w img_h];
ax2 = axes('Parent', fig2, 'Units', 'normalized', 'Position', [0 0 1 1]);
imshow(uint8(min(255, star_info.ideal_gray)), 'Parent', ax2, 'InitialMagnification', 100);
axis(ax2, 'off');

% Figure 3: 별 위치 오버레이 (분석용)
fig3 = figure('Position', [150 50 1400 500], 'Name', '시뮬레이션 분석', 'Color', 'k');

subplot(1,3,1);
imshow(uint8(min(255, star_info.ideal_gray)), []);
title('이상적 Grayscale', 'Color', 'w', 'FontSize', 12);

subplot(1,3,2);
imshow(uint8(min(255, star_info.ideal_gray)), []);
hold on;
if star_info.num_stars > 0
    plot(star_info.true_centroids(:,1), star_info.true_centroids(:,2), ...
        'ro', 'MarkerSize', 12, 'LineWidth', 2);

    % 밝은 별에 등급 레이블 표시
    % --- 밝은 별 등급 레이블 표시 ---
    % 가장 밝은 별 상위 10개에 등급(magnitude) 값을 텍스트로 표시
    % sort(): 등급을 오름차순 정렬 (작은 값 = 밝은 별)
    %   천문학 등급 체계: 숫자가 작을수록 밝음 (1등급 > 6등급)
    % +10 pixel X 오프셋: 별 마커와 텍스트가 겹치지 않도록 우측에 표시
    % 노란색: 검정 배경에서 가시성이 가장 좋은 색상
    for i = 1:min(10, star_info.num_stars)
        [~, idx] = sort(star_info.magnitudes);
        bright_idx = idx(i);
        text(star_info.true_centroids(bright_idx,1)+10, ...
             star_info.true_centroids(bright_idx,2), ...
             sprintf('%.1f', star_info.magnitudes(bright_idx)), ...
             'Color', 'yellow', 'FontSize', 9);
    end
end
hold off;
title(sprintf('별 위치 표시 (%d개)', star_info.num_stars), 'Color', 'w', 'FontSize', 12);

subplot(1,3,3);
imshow(bayer_img, []);
title('Bayer 패턴 + 노이즈', 'Color', 'w', 'FontSize', 12);

sgtitle(sprintf('Star Tracker (RA=%.1f°, DEC=%.1f°, FOV=%.1f°×%.1f°)', ...
    ra_deg, de_deg, FOVx, FOVy), 'FontSize', 14, 'Color', 'w');

% Figure 4: 등급별 별 분포
if star_info.num_stars > 0
    fig4 = figure('Position', [200 150 600 400], 'Name', '등급 분포');
    histogram(star_info.magnitudes, 'BinWidth', 0.5, 'FaceColor', [0.3 0.5 0.8]);
    xlabel('등급 (Magnitude)');
    ylabel('별 개수');
    title(sprintf('FOV 내 별 등급 분포 (총 %d개)', star_info.num_stars));
    grid on;

    % 등급별 플럭스 참고선
    % --- Pogson 공식 기반 플럭스 참조 곡선 ---
    % 2축 그래프: 왼쪽=별 개수(히스토그램), 오른쪽=플럭스(선 그래프)
    % Pogson 공식: flux = ref_flux × 10^(-0.4 × (mag - ref_mag))
    %   ref_flux = 1000 [ADU] (6등급 기준, 시뮬레이션 정규화 값)
    %   ref_mag = 6.0 (기준 등급)
    %   0등급: 1000 × 10^(-0.4×(-6)) = 1000 × 251 ≈ 251,000 ADU
    %   3등급: 1000 × 10^(-0.4×(-3)) = 1000 × 15.8 ≈ 15,800 ADU
    %   6등급: 1000 × 10^0 = 1,000 ADU (기준)
    % 이 곡선은 히스토그램과 함께 보면 어두운 별이 많음을 시각적으로 확인 가능
    yyaxis right;
    mags = 0:0.5:7;
    ref_flux = 1000; ref_mag = 6.0;
    fluxes = ref_flux * 10.^(-0.4 * (mags - ref_mag));
    plot(mags, fluxes, 'r-', 'LineWidth', 2);
    ylabel('상대 플럭스 (ADU)');
    legend('별 개수', 'Pogson 플럭스', 'Location', 'northeast');
end

%% ========== 4. 별 정보 테이블 ==========
fprintf('4. 밝은 별 정보 (상위 10개):\n');
fprintf('   %-4s  %-8s  %-8s  %-6s  %-10s  %-8s  %-8s\n', ...
    '순위', 'X (px)', 'Y (px)', '등급', 'photon/s', 'e-', 'ADU');
fprintf('   %-4s  %-8s  %-8s  %-6s  %-10s  %-8s  %-8s\n', ...
    '----', '------', '------', '----', '--------', '------', '------');

if star_info.num_stars > 0
    [sorted_mag, idx] = sort(star_info.magnitudes);
    ref_flux = 1000; ref_mag = 6.0;

    for i = 1:min(10, star_info.num_stars)
        j = idx(i);
        photon_flux = ref_flux * 10^(-0.4 * (star_info.magnitudes(j) - ref_mag));
        electrons = photon_flux * sensor_params.exposure_time * sensor_params.quantum_efficiency;
        adu = electrons * sensor_params.analog_gain * sensor_params.digital_gain;
        fprintf('   %-4d  %-8.1f  %-8.1f  %-6.2f  %-10.0f  %-8.1f  %-8.0f\n', ...
            i, star_info.true_centroids(j,1), star_info.true_centroids(j,2), ...
            star_info.magnitudes(j), photon_flux, electrons, adu);
    end
end
fprintf('\n');

%% ========== 5. 결과 저장 ==========
fprintf('5. 결과 저장...\n');

% 실제 크기 이미지 저장 (1:1 픽셀)
imwrite(uint8(min(255, star_info.ideal_gray)), fullfile(output_dir, 'star_image_ideal.png'));
imwrite(bayer_img, fullfile(output_dir, 'star_image_bayer.png'));

% Figure 저장
saveas(fig3, fullfile(output_dir, 'star_simulation_analysis.png'));

if star_info.num_stars > 0
    saveas(fig4, fullfile(output_dir, 'magnitude_distribution.png'));
end

% 별 정보 MAT 파일 저장
save(fullfile(output_dir, 'star_info.mat'), 'star_info', 'sensor_params', 'ra_deg', 'de_deg', 'roll_deg');

fprintf('   저장 완료: %s\n\n', output_dir);

%% ========== 6. 요약 ==========
fprintf('================================================\n');
fprintf('  시뮬레이션 완료\n');
fprintf('================================================\n\n');

fprintf('생성된 이미지:\n');
fprintf('  - star_image_ideal.png : 이상적 Grayscale\n');
fprintf('  - star_image_bayer.png : Bayer 패턴 (노이즈 포함)\n');
fprintf('  - star_info.mat : 별 정보 데이터\n\n');

fprintf('추가 분석:\n');
fprintf('  - Bayer->Gray 변환 비교: sub_main_1_bayer_comparison.m 실행\n');
fprintf('================================================\n');

%% ========== 7. 우주 vs 지상 환경 비교 분석 ==========
fprintf('\n\n');
fprintf('================================================\n');
fprintf('  우주 vs 지상 환경 비교 분석\n');
fprintf('================================================\n\n');

% 두 환경에서 동일 관측 방향으로 이미지 생성
env_list = {'ground', 'space'};
env_temps = [25, -20];  % 지상: 25 deg C, 우주: -20 deg C
env_imgs = cell(1, 2);
env_bayers = cell(1, 2);
env_snrs = zeros(1, 2);

for ei = 1:2
    sp = sensor_params;
    sp.environment = env_list{ei};
    sp.sensor_temp = env_temps(ei);
    if strcmp(env_list{ei}, 'ground')
        sp.cosmic_ray_rate = 0;
        sp.stray_light = 0;
        sp.dsnu = 0;
        sp.prnu = 0;
    end
    [g, b, ~] = simulate_star_image_realistic(ra_deg, de_deg, roll_deg, sp);
    env_imgs{ei} = g;
    env_bayers{ei} = b;
    env_snrs(ei) = calculate_peak_snr(double(g));
end

% --- Figure: 4-panel 비교 ---
fig_cmp = figure('Position', [100 100 1400 700], 'Name', 'Ground vs Space', 'Color', 'k');

subplot(2,2,1);
imshow(env_imgs{1}, []);
title(sprintf('Ground Gray (25 deg C)\nPeak SNR = %.1f dB', env_snrs(1)), ...
    'Color', 'w', 'FontSize', 11);

subplot(2,2,2);
imshow(env_imgs{2}, []);
title(sprintf('Space Gray (-20 deg C)\nPeak SNR = %.1f dB', env_snrs(2)), ...
    'Color', 'w', 'FontSize', 11);

subplot(2,2,3);
imshow(env_bayers{1}, []);
title('Ground Bayer', 'Color', 'w', 'FontSize', 11);

subplot(2,2,4);
imshow(env_bayers{2}, []);
title('Space Bayer', 'Color', 'w', 'FontSize', 11);

sgtitle(sprintf('Ground vs Space (RA=%.1f, DEC=%.1f)', ra_deg, de_deg), ...
    'FontSize', 14, 'Color', 'w');

saveas(fig_cmp, fullfile(output_dir, 'ground_vs_space_comparison.png'));

% --- Figure: 배경 픽셀 히스토그램 비교 ---
fig_hist = figure('Position', [200 100 900 400], 'Name', 'Background Histogram');

subplot(1,2,1);
histogram(double(env_imgs{1}(:)), 0:255, 'FaceColor', [0.8 0.4 0.2], 'EdgeColor', 'none');
hold on;
histogram(double(env_imgs{2}(:)), 0:255, 'FaceColor', [0.2 0.5 0.9], 'EdgeColor', 'none');
hold off;
xlabel('Pixel Value (ADU)');
ylabel('Count');
title('Background Distribution');
legend('Ground (25 deg C)', 'Space (-20 deg C)', 'Location', 'northeast');
set(gca, 'YScale', 'log');
xlim([0 30]);
grid on;

subplot(1,2,2);
% 노이즈 기여도 바 차트
dark_ref = sensor_params.dark_current_ref;
T_double = sensor_params.dark_temp_double;
T_ref = sensor_params.dark_temp_ref;
gain = sensor_params.analog_gain * sensor_params.digital_gain;
exp_t = sensor_params.exposure_time;

dark_ground = dark_ref * exp_t;
dark_space = dark_ref * 2^((-20 - T_ref) / T_double) * exp_t;
read_e = sensor_params.read_noise;
stray_e = sensor_params.stray_light * exp_t;
dsnu_e = sensor_params.dsnu * exp_t;

noise_labels = {'Shot(6mag)', 'Read', 'Dark(25C)', 'Dark(-20C)', 'Stray', 'DSNU'};
% 6등급 별 신호 기준 shot noise 추정
if strcmp(sensor_params.environment, 'space')
    ref_flux = 96 / 0.56;
else
    ref_flux = 96;
end
sig_6mag = ref_flux * exp_t * sensor_params.quantum_efficiency;
noise_vals = [sqrt(sig_6mag), read_e, sqrt(dark_ground), sqrt(dark_space), ...
    sqrt(max(0, stray_e)), dsnu_e];

bar_colors = [0.9 0.6 0.2; 0.3 0.6 0.9; 0.8 0.3 0.3; 0.2 0.4 0.8; 0.5 0.8 0.5; 0.7 0.5 0.7];
b = bar(noise_vals, 'FaceColor', 'flat');
b.CData = bar_colors;
set(gca, 'XTickLabel', noise_labels, 'XTickLabelRotation', 30);
ylabel('Noise [e- RMS]');
title('Noise Budget per Frame');
grid on;

saveas(fig_hist, fullfile(output_dir, 'noise_analysis.png'));

% --- 텍스트 요약 ---
fprintf('노이즈 기여도 분석 (per frame, %.1f ms):\n', exp_t * 1000);
fprintf('  %-20s  %-12s  %-12s\n', '', 'Ground', 'Space');
fprintf('  %-20s  %-12s  %-12s\n', '----', '------', '------');
fprintf('  %-20s  %.4f e-     %.6f e-\n', 'Dark current', dark_ground, dark_space);
fprintf('  %-20s  %.1f e-        %.1f e-\n', 'Read noise', read_e, read_e);
fprintf('  %-20s  %.3f e-      %.3f e-\n', 'Shot(6mag)', sqrt(sig_6mag), sqrt(sig_6mag));
fprintf('  %-20s  -             %.4f e-\n', 'Stray light', stray_e);
fprintf('  %-20s  -             %.6f e-\n', 'DSNU', dsnu_e * 2^((-20 - T_ref) / T_double));
fprintf('  %-20s  -             1%%\n\n', 'PRNU');
fprintf('  Peak SNR (Ground):  %.1f dB\n', env_snrs(1));
fprintf('  Peak SNR (Space):   %.1f dB\n', env_snrs(2));
fprintf('  SNR 개선:           +%.1f dB\n\n', env_snrs(2) - env_snrs(1));

fprintf('저장: %s\n', fullfile(output_dir, 'ground_vs_space_comparison.png'));
fprintf('저장: %s\n', fullfile(output_dir, 'noise_analysis.png'));
fprintf('================================================\n');
