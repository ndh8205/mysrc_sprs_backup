%% test_all_sensor_combinations.m
% 모든 센서+렌즈 조합 별 이미지 시뮬레이션 테스트
%
% 목적:
%   IMX296 / IMX477 / IMX219 세 센서와 6종 렌즈의 전체 조합을
%   자동으로 테스트하고, FOV/SNR/롤링셔터 효과를 비교 분석합니다.
%
% 테스트 항목:
%   1. 각 센서+렌즈 조합의 이미지 생성
%   2. FOV, 검출 별 수, Peak SNR 비교
%   3. 롤링셔터 vs 글로벌셔터 별 위치 왜곡 분석
%   4. 센서별 노이즈 특성 비교
%
% 출력:
%   - output/sensor_comparison/ 폴더에 이미지 및 분석 결과
%   - 콘솔에 비교 테이블 출력
%
% 의존성:
%   - get_sensor_preset.m (센서/렌즈 프리셋)
%   - simulate_star_image_realistic.m (시뮬레이션 엔진)
%   - calculate_peak_snr.m (SNR 계산)

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

%% 출력 폴더
output_dir = fullfile(script_dir, 'output', 'sensor_comparison');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% 관측 방향 (오리온 벨트)
ra_deg = 84;
de_deg = -1;
roll_deg = 0;

%% 센서/렌즈 조합 정의
sensors = {'IMX296', 'IMX477', 'IMX219'};
lenses = {'6mm', '8mm', '12mm', '16mm', '25mm', '8-50mm'};

n_sensors = length(sensors);
n_lenses = length(lenses);
n_total = n_sensors * n_lenses;

fprintf('================================================================\n');
fprintf('  센서+렌즈 전체 조합 테스트 (%d 조합)\n', n_total);
fprintf('================================================================\n');
fprintf('  센서: %s\n', strjoin(sensors, ', '));
fprintf('  렌즈: %s\n', strjoin(lenses, ', '));
fprintf('  관측 방향: RA=%.1f, DEC=%.1f, Roll=%.1f\n', ra_deg, de_deg, roll_deg);
fprintf('================================================================\n\n');

%% 결과 저장 구조체
results = struct();
results.sensors = sensors;
results.lenses = lenses;
results.FOVx = zeros(n_sensors, n_lenses);
results.FOVy = zeros(n_sensors, n_lenses);
results.n_stars = zeros(n_sensors, n_lenses);
results.peak_snr = zeros(n_sensors, n_lenses);
results.resolution = cell(n_sensors, n_lenses);
results.pixel_size = zeros(n_sensors, n_lenses);
results.psf_sigma = zeros(n_sensors, n_lenses);
results.gen_time = zeros(n_sensors, n_lenses);

%% 전체 조합 테스트
combo_idx = 0;
for si = 1:n_sensors
    for li = 1:n_lenses
        combo_idx = combo_idx + 1;
        sensor_name = sensors{si};
        lens_name = lenses{li};

        fprintf('[%2d/%d] %s + %s ... ', combo_idx, n_total, sensor_name, lens_name);

        % 프리셋 로드
        sp = get_sensor_preset(sensor_name, lens_name);

        % 시뮬레이션 실행
        tic;
        [gray_img, bayer_img, star_info] = simulate_star_image_realistic(...
            ra_deg, de_deg, roll_deg, sp);
        gen_time = toc;

        % 결과 저장
        results.FOVx(si, li) = sp.FOVx;
        results.FOVy(si, li) = sp.FOVy;
        results.n_stars(si, li) = star_info.num_stars;
        results.peak_snr(si, li) = calculate_peak_snr(double(gray_img));
        results.resolution{si, li} = sprintf('%dx%d', sp.l, sp.w);
        results.pixel_size(si, li) = sp.myu * 1e6;  % [um]
        results.psf_sigma(si, li) = sp.psf_sigma;
        results.gen_time(si, li) = gen_time;

        fprintf('FOV=%.1fx%.1f, %d stars, SNR=%.1f dB, %.1fs\n', ...
            sp.FOVx, sp.FOVy, star_info.num_stars, ...
            results.peak_snr(si, li), gen_time);

        % 이미지 저장
        img_name = sprintf('%s_%s_gray.png', sensor_name, lens_name);
        % uint16 -> uint8 변환 (표시/저장용)
        gray_8bit = uint8(double(gray_img) / double(max(gray_img(:))) * 255);
        imwrite(gray_8bit, fullfile(output_dir, img_name));
    end
    fprintf('\n');
end

%% ========== 결과 비교 테이블 ==========
fprintf('\n================================================================\n');
fprintf('  결과 비교 테이블\n');
fprintf('================================================================\n\n');

% 헤더
fprintf('%-8s %-8s  %-12s  %-12s  %-6s  %-8s  %-7s  %-8s\n', ...
    'Sensor', 'Lens', 'Resolution', 'FOV [deg]', 'Stars', 'SNR[dB]', 'PSF[px]', 'Time[s]');
fprintf('%-8s %-8s  %-12s  %-12s  %-6s  %-8s  %-7s  %-8s\n', ...
    '------', '------', '----------', '---------', '-----', '-------', '------', '-------');

for si = 1:n_sensors
    for li = 1:n_lenses
        fprintf('%-8s %-8s  %-12s  %5.1fx%4.1f   %4d   %6.1f   %5.2f   %6.2f\n', ...
            sensors{si}, lenses{li}, results.resolution{si, li}, ...
            results.FOVx(si, li), results.FOVy(si, li), ...
            results.n_stars(si, li), results.peak_snr(si, li), ...
            results.psf_sigma(si, li), results.gen_time(si, li));
    end
    fprintf('\n');
end

%% ========== 롤링셔터 vs 글로벌셔터 비교 ==========
fprintf('================================================================\n');
fprintf('  롤링셔터 vs 글로벌셔터 비교\n');
fprintf('================================================================\n\n');

% 위성 자세 변화 속도 시나리오
omega_scenarios = {
    '정지 (0 deg/s)',      [0, 0, 0];
    '저속 (0.5 deg/s)',    [0, 0.5, 0];
    '중속 (2 deg/s)',      [0, 2.0, 0];
    '고속 (5 deg/s)',      [0, 5.0, 0];
};

% IMX477 (롤링셔터) + 16mm 렌즈로 테스트
test_sensor = 'IMX477';
test_lens = '16mm';

fprintf('센서: %s + %s (readout_time=32ms)\n\n', test_sensor, test_lens);
fprintf('%-20s  %-12s  %-12s  %-10s\n', ...
    'Scenario', 'Max Shift', 'SNR [dB]', 'Stars');
fprintf('%-20s  %-12s  %-12s  %-10s\n', ...
    '--------', '---------', '--------', '-----');

rs_images = cell(size(omega_scenarios, 1), 1);

for oi = 1:size(omega_scenarios, 1)
    sp = get_sensor_preset(test_sensor, test_lens, ...
        'angular_velocity', omega_scenarios{oi, 2});

    [gray_rs, ~, info_rs] = simulate_star_image_realistic(...
        ra_deg, de_deg, roll_deg, sp);

    snr_rs = calculate_peak_snr(double(gray_rs));
    rs_images{oi} = gray_rs;

    fprintf('%-20s  %8.2f px   %8.1f     %4d\n', ...
        omega_scenarios{oi, 1}, info_rs.max_rs_shift_px, snr_rs, info_rs.num_stars);
end

% 글로벌셔터 비교 (IMX296 + 같은 렌즈)
sp_gs = get_sensor_preset('IMX296', test_lens, ...
    'angular_velocity', omega_scenarios{4, 2});  % 고속
[gray_gs, ~, info_gs] = simulate_star_image_realistic(ra_deg, de_deg, roll_deg, sp_gs);
snr_gs = calculate_peak_snr(double(gray_gs));
fprintf('\n%-20s  %8.2f px   %8.1f     %4d  (IMX296 GS)\n', ...
    'GS 고속 (5 deg/s)', info_gs.max_rs_shift_px, snr_gs, info_gs.num_stars);

%% ========== 시각화 ==========
fprintf('\n시각화 생성 중...\n');

% --- Figure 1: 전 조합 그리드 ---
fig1 = figure('Position', [50 50 1800 900], 'Name', 'All Combinations', 'Color', 'k');
for si = 1:n_sensors
    for li = 1:n_lenses
        idx = (si-1) * n_lenses + li;
        subplot(n_sensors, n_lenses, idx);

        % 이미지 로드 (저장된 것)
        img_name = sprintf('%s_%s_gray.png', sensors{si}, lenses{li});
        img = imread(fullfile(output_dir, img_name));

        imshow(img, []);
        title(sprintf('%s+%s\nFOV=%.1fx%.1f, %d stars', ...
            sensors{si}, lenses{li}, ...
            results.FOVx(si, li), results.FOVy(si, li), ...
            results.n_stars(si, li)), ...
            'Color', 'w', 'FontSize', 8);
    end
end
sgtitle(sprintf('센서+렌즈 전체 조합 (RA=%.0f, DEC=%.0f)', ra_deg, de_deg), ...
    'Color', 'w', 'FontSize', 14);
saveas(fig1, fullfile(output_dir, 'all_combinations_grid.png'));

% --- Figure 2: 롤링셔터 비교 ---
fig2 = figure('Position', [100 100 1400 400], 'Name', 'Rolling Shutter Effect', 'Color', 'k');
for oi = 1:size(omega_scenarios, 1)
    subplot(1, size(omega_scenarios, 1), oi);
    gray_8bit = uint8(double(rs_images{oi}) / double(max(rs_images{oi}(:))) * 255);
    imshow(gray_8bit, []);
    title(sprintf('%s\nShift=%.1f px', ...
        omega_scenarios{oi, 1}, ...
        max(abs(omega_scenarios{oi, 2})) * deg2rad(1) * ...
        (0.016 / 1.55e-6) * 0.032), ...
        'Color', 'w', 'FontSize', 10);
end
sgtitle(sprintf('Rolling Shutter Effect (%s + %s)', test_sensor, test_lens), ...
    'Color', 'w', 'FontSize', 14);
saveas(fig2, fullfile(output_dir, 'rolling_shutter_comparison.png'));

% --- Figure 3: FOV vs 별 수 산점도 ---
fig3 = figure('Position', [150 150 800 500], 'Name', 'FOV vs Stars');
colors = [0.9 0.2 0.2; 0.2 0.6 0.9; 0.2 0.8 0.3];
markers = {'o', 's', 'd'};
hold on;
for si = 1:n_sensors
    fov_diag = sqrt(results.FOVx(si,:).^2 + results.FOVy(si,:).^2);
    scatter(fov_diag, results.n_stars(si,:), 100, colors(si,:), ...
        markers{si}, 'filled', 'DisplayName', sensors{si});
    % 렌즈 레이블
    for li = 1:n_lenses
        text(fov_diag(li)+0.3, results.n_stars(si,li)+1, lenses{li}, ...
            'FontSize', 7, 'Color', colors(si,:));
    end
end
hold off;
xlabel('Diagonal FOV [deg]');
ylabel('Number of Stars');
title('FOV vs Detected Stars (all combinations)');
legend('Location', 'northwest');
grid on;
saveas(fig3, fullfile(output_dir, 'fov_vs_stars.png'));

% --- Figure 4: SNR 비교 바 차트 ---
fig4 = figure('Position', [200 200 1000 400], 'Name', 'SNR Comparison');
bar_data = results.peak_snr';
b = bar(bar_data, 'grouped');
for si = 1:n_sensors
    b(si).FaceColor = colors(si,:);
end
set(gca, 'XTickLabel', lenses);
xlabel('Lens');
ylabel('Peak SNR [dB]');
title('Peak SNR: Sensor x Lens');
legend(sensors, 'Location', 'northeast');
grid on;
saveas(fig4, fullfile(output_dir, 'snr_comparison.png'));

%% ========== 결과 MAT 파일 저장 ==========
save(fullfile(output_dir, 'comparison_results.mat'), 'results', ...
    'ra_deg', 'de_deg', 'roll_deg', 'omega_scenarios');

fprintf('\n================================================================\n');
fprintf('  테스트 완료! 결과: %s\n', output_dir);
fprintf('================================================================\n');
