function gui_star_simulator()
% GUI_STAR_SIMULATOR Star Tracker 시뮬레이션 인터랙티브 GUI
%
% 사용법:
%   gui_star_simulator
%
% 기능:
%   - 센서/노출/관측 파라미터를 슬라이더/프리셋으로 조절
%   - 이상적 Grayscale vs 변환된 Grayscale 이미지 비교
%   - FOV, SNR, 별 검출률, Centroid 정확도 등 실시간 메트릭
%   - 5가지 Bayer→Gray 변환 방법 비교 (optimal 포함)
%
% 의존성:
%   core/simulate_star_image_realistic.m
%   core/bayer_to_gray_direct.m
%   utils/detect_stars_simple.m
%   utils/calculate_peak_snr.m
%   utils/evaluate_centroid_accuracy.m

%% 경로 설정 (프로젝트 / 포터블 자동 감지)
script_dir = fileparts(mfilename('fullpath'));
core_dir = fullfile(script_dir, 'core');
if exist(core_dir, 'dir')
    % 포터블 모드: core/ 폴더 사용
    addpath(core_dir);
else
    % 프로젝트 모드: bayer_research/ → study/ → Space_SLAM/
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model', 'sensor', 'star_tracker', 'bayer'));
end

%% 카탈로그 사전 로드
catalog = preload_catalog(script_dir);

%% 상태 초기화
app = struct();
app.catalog = catalog;
app.params = init_params();
app.cache = struct('gray_ideal', [], 'bayer_img', [], 'star_info', [], ...
    'gray_converted', [], 'method_info', [], ...
    'detection', [], 'snr_db', 0, 'centroid_rms', 0);
app.dirty = struct('stage1', true, 'stage2', true, 'stage3', true);

%% 화면 크기 감지 → 레이아웃 스케일링
scr = get(0, 'ScreenSize');
fig_w = min(1600, round(scr(3) * 0.92));
fig_h = min(900, round(scr(4) * 0.85));
fig_x = max(10, round((scr(3) - fig_w) / 2));
fig_y = max(30, round((scr(4) - fig_h) / 2));
app.fig_w = fig_w;
app.fig_h = fig_h;

%% 메인 윈도우
app.fig = uifigure('Name', 'Star Tracker Simulator', ...
    'Position', [fig_x fig_y fig_w fig_h], 'Color', [0.15 0.15 0.18], ...
    'Resize', 'off');

%% 좌측 패널: 탭
left_panel = uipanel(app.fig, 'Position', [0 0 400 fig_h], ...
    'BackgroundColor', [0.18 0.18 0.22], 'BorderType', 'none');
app.tabgroup = uitabgroup(left_panel, 'Position', [5 5 390 fig_h-10]);

%% 탭 생성
app = create_sensor_tab(app);
app = create_exposure_tab(app);
app = create_noise_tab(app);
app = create_observation_tab(app);
app = create_motion_tab(app);
app = create_processing_tab(app);

%% 우측: 버튼, 디스플레이 컨트롤, 이미지, 메트릭
app = create_action_buttons(app);
app = create_display_controls(app);
app = create_image_area(app);
app = create_metrics_panel(app);

%% 초기 FOV/노이즈 계산
update_fov_display(app);
update_noise_budget(app);

%% 저장
app.fig.UserData = app;
end

%% ========== 카탈로그 사전 로드 ==========
function catalog = preload_catalog(script_dir)
    % 포터블: script_dir/data/, 프로젝트: Space_SLAM/data/
    data_dir = fullfile(script_dir, 'data');
    if ~exist(data_dir, 'dir')
        project_root = fileparts(fileparts(script_dir));
        data_dir = fullfile(project_root, 'data');
    end
    mat_path = fullfile(data_dir, 'star_catalog_kvector.mat');
    csv_path = fullfile(data_dir, 'Hipparcos_Below_6.0.csv');

    catalog = [];
    if exist(mat_path, 'file')
        fprintf('카탈로그 로드 중 (star_catalog_kvector.mat)...\n');
        data = load(mat_path);
        catalog = data.catalog_data;
        fprintf('카탈로그 로드 완료 (%d stars)\n', length(catalog.star_catalog.RA));
    elseif exist(csv_path, 'file')
        fprintf('카탈로그 로드 중 (CSV)...\n');
        opts = detectImportOptions(csv_path);
        tbl = readtable(csv_path, opts);
        catalog.star_catalog.RA = tbl.RA;
        catalog.star_catalog.DEC = tbl.DE;
        catalog.star_catalog.Magnitude = tbl.Magnitude;
        fprintf('CSV 카탈로그 로드 완료\n');
    else
        warning('별 카탈로그를 찾을 수 없습니다. data/ 폴더를 확인하세요.');
    end
end

%% ========== 기본 파라미터 ==========
function p = init_params()
    % Sensor
    p.pixel_size = 2.0;         % [um]
    p.focal_length = 10.42;     % [mm]
    p.res_w = 1280;             % [px]
    p.res_h = 720;              % [px]
    p.qe = 0.5;                 % [0~1]
    p.mag_limit = 6.5;

    % Exposure/Gain
    p.exposure_ms = 22.0;       % [ms]
    p.analog_gain = 16.0;       % [x]
    p.digital_gain = 1.0;       % [x]

    % Noise
    p.noise_enabled = true;
    p.dark_current = 0.1;       % [e-/px/s]
    p.read_noise = 3.0;         % [e- RMS]

    % Bayer channel sensitivity
    p.sens_R = 1.0;
    p.sens_G = 1.0;
    p.sens_B = 0.9;

    % Observation
    p.ra = 84.0;                % [deg]
    p.dec = -1.0;               % [deg]
    p.roll = 0.0;               % [deg]

    % Processing
    p.conv_method = 'raw';
    p.opt_weights = [0.4544, 0.3345, 0.2111];
    p.threshold = 15;           % [ADU]
    p.min_area = 2;             % [px]

    % Display
    p.show_overlay = false;
    p.show_labels = false;

    % Display Pipeline (RAW→Display 변환)
    p.display_mode = 'RAW';        % 'RAW' | 'Digital Gain' | 'Window' | 'Gamma'
    p.display_dgain = 1;           % 디스플레이 디지털 게인 (1,2,4,8,16)
    p.display_black = 0;           % Window 모드: black level [ADU]
    p.display_white = 255;         % Window 모드: white level [ADU] (기본 8-bit)
    p.display_gamma = 0.5;         % Gamma 모드: gamma 값 (0.1~1.0)
    p.show_histogram = false;      % 히스토그램 표시 여부

    % Sensor/Lens Preset
    p.sensor_preset = 'OV4689';
    p.lens_preset = 'Custom';

    % Motion (angular velocity)
    p.omega_x = 0;              % [deg/s]
    p.omega_y = 0;              % [deg/s]
    p.omega_z = 0;              % [deg/s]

    % Advanced sensor params
    p.shutter_type = 'global';
    p.readout_time = 0;         % [s]
    p.adc_bits = 8;
    p.full_well = 10000;        % [e-]
    p.psf_sigma = 1.2;          % [px]

    % Environment
    p.environment = 'space';
    p.sensor_temp = -20;        % [C]

    % Preview scale (1=full, 2=half, 4=quarter)
    p.preview_scale = 1;
end

%% ========== Tab 1: Sensor ==========
function app = create_sensor_tab(app)
    tab = uitab(app.tabgroup, 'Title', 'Sensor', ...
        'BackgroundColor', [0.2 0.2 0.24], 'Scrollable', 'on');

    y = 835;

    % --- Sensor Preset ---
    uilabel(tab, 'Position', [10 y 120 20], 'Text', 'Sensor Preset', ...
        'FontColor', [1 0.9 0.4], 'FontWeight', 'bold');
    sensor_list = {'OV4689', ...
        '--- Starvis 2 (BSI) ---', 'IMX585', 'IMX678', 'IMX662', ...
        '--- Starvis (BSI) ---', 'IMX485', 'IMX462', ...
        '--- Pregius (GS) ---', 'IMX174', 'IMX249', 'IMX264', 'IMX250', 'IMX252', ...
        '--- Sony (기타) ---', 'IMX296', 'IMX477', 'IMX219', ...
        '--- ams-OSRAM (GS) ---', 'CMV4000', 'CMV2000', ...
        '--- OmniVision ---', 'OV9281', ...
        '--- onsemi ---', 'MT9P031', 'AR0134', ...
        'Custom'};
    app.h.dd_sensor = uidropdown(tab, 'Position', [130 y-2 235 24], ...
        'Items', sensor_list, ...
        'Value', app.params.sensor_preset, ...
        'BackgroundColor', [0.25 0.25 0.3]);
    app.h.dd_sensor.ValueChangedFcn = @(s,e) cb_sensor_preset(s, e, app.fig);

    % --- Lens Preset ---
    y = y - 30;
    uilabel(tab, 'Position', [10 y 120 20], 'Text', 'Lens Preset', ...
        'FontColor', [1 0.9 0.4], 'FontWeight', 'bold');
    app.h.dd_lens = uidropdown(tab, 'Position', [130 y-2 235 24], ...
        'Items', {'Custom', '6mm', '8mm', '12mm', '16mm', '25mm', '8-50mm'}, ...
        'Value', app.params.lens_preset, ...
        'BackgroundColor', [0.25 0.25 0.3]);
    app.h.dd_lens.ValueChangedFcn = @(s,e) cb_lens_preset(s, e, app.fig);

    % --- Sensor Info Display ---
    y = y - 28;
    app.h.lbl_sensor_info = uilabel(tab, 'Position', [10 y 370 18], ...
        'Text', 'OV4689 | GS | 8-bit | FW=10000e-', ...
        'FontColor', [0.6 0.8 1], 'FontName', 'Consolas', 'FontSize', 10);

    % --- Pixel Size ---
    y = y - 30;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Pixel Size [um]', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 28;
    app.h.slider_pix = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [1 10], 'Value', app.params.pixel_size, 'MajorTicks', 1:10);
    app.h.edit_pix = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.pixel_size, 'Limits', [1 10], ...
        'ValueDisplayFormat', '%.2f');
    app.h.slider_pix.ValueChangedFcn = @(s,e) cb_sensor(s, e, 'pixel_size', app.fig);
    app.h.edit_pix.ValueChangedFcn = @(s,e) cb_sensor_edit(s, e, 'pixel_size', app.fig);

    % --- Focal Length ---
    y = y - 45;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Focal Length [mm]', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 28;
    app.h.slider_fl = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [5 50], 'Value', app.params.focal_length, 'MajorTicks', [5 10 20 30 40 50]);
    app.h.edit_fl = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.focal_length, 'Limits', [5 50], ...
        'ValueDisplayFormat', '%.2f');
    app.h.slider_fl.ValueChangedFcn = @(s,e) cb_sensor(s, e, 'focal_length', app.fig);
    app.h.edit_fl.ValueChangedFcn = @(s,e) cb_sensor_edit(s, e, 'focal_length', app.fig);

    % --- Resolution + Preview Scale ---
    y = y - 45;
    uilabel(tab, 'Position', [10 y 100 20], 'Text', 'Resolution', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    uilabel(tab, 'Position', [120 y 80 20], 'Text', 'Preview:', ...
        'FontColor', [0.7 0.7 0.7]);
    app.h.btn_scale1 = uibutton(tab, 'Position', [185 y-2 45 22], 'Text', '1x', ...
        'FontSize', 10, 'ButtonPushedFcn', @(s,e) cb_preview_scale(s, e, 1, app.fig));
    app.h.btn_scale2 = uibutton(tab, 'Position', [233 y-2 45 22], 'Text', '1/2', ...
        'FontSize', 10, 'ButtonPushedFcn', @(s,e) cb_preview_scale(s, e, 2, app.fig));
    app.h.btn_scale4 = uibutton(tab, 'Position', [281 y-2 45 22], 'Text', '1/4', ...
        'FontSize', 10, 'BackgroundColor', [0.3 0.5 0.3], ...
        'ButtonPushedFcn', @(s,e) cb_preview_scale(s, e, 4, app.fig));
    y = y - 28;
    uilabel(tab, 'Position', [10 y 30 22], 'Text', 'W:', 'FontColor', [0.7 0.7 0.7]);
    app.h.edit_resw = uieditfield(tab, 'numeric', 'Position', [35 y 70 22], ...
        'Value', app.params.res_w, 'Limits', [100 5000], 'RoundFractionalValues', 'on');
    uilabel(tab, 'Position', [120 y 30 22], 'Text', 'H:', 'FontColor', [0.7 0.7 0.7]);
    app.h.edit_resh = uieditfield(tab, 'numeric', 'Position', [145 y 70 22], ...
        'Value', app.params.res_h, 'Limits', [100 4000], 'RoundFractionalValues', 'on');
    app.h.lbl_preview = uilabel(tab, 'Position', [225 y 150 22], ...
        'Text', '', 'FontColor', [1 0.8 0.3], 'FontName', 'Consolas', 'FontSize', 10);
    app.h.edit_resw.ValueChangedFcn = @(s,e) cb_res_changed(s, e, app.fig);
    app.h.edit_resh.ValueChangedFcn = @(s,e) cb_res_changed(s, e, app.fig);

    % --- Quantum Efficiency ---
    y = y - 35;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Quantum Efficiency', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 28;
    app.h.slider_qe = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [0.1 0.9], 'Value', app.params.qe, 'MajorTicks', 0.1:0.1:0.9);
    app.h.edit_qe = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.qe, 'Limits', [0.1 0.9], ...
        'ValueDisplayFormat', '%.2f');
    app.h.slider_qe.ValueChangedFcn = @(s,e) cb_sensor(s, e, 'qe', app.fig);
    app.h.edit_qe.ValueChangedFcn = @(s,e) cb_sensor_edit(s, e, 'qe', app.fig);

    % --- Magnitude Limit ---
    y = y - 45;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Star Magnitude Limit', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 28;
    app.h.slider_mag = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [3 8], 'Value', app.params.mag_limit, 'MajorTicks', 3:8);
    app.h.edit_mag = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.mag_limit, 'Limits', [3 8], ...
        'ValueDisplayFormat', '%.1f');
    app.h.slider_mag.ValueChangedFcn = @(s,e) cb_sensor(s, e, 'mag_limit', app.fig);
    app.h.edit_mag.ValueChangedFcn = @(s,e) cb_sensor_edit(s, e, 'mag_limit', app.fig);

    % --- FOV Display ---
    y = y - 50;
    fov_panel = uipanel(tab, 'Position', [10 y-80 360 100], ...
        'Title', 'Calculated FOV', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.15 0.2 0.15], 'ForegroundColor', [0.5 1 0.5]);
    app.h.lbl_fov_h = uilabel(fov_panel, 'Position', [10 55 340 20], ...
        'Text', 'H: --', 'FontColor', [0.5 1 0.5], 'FontName', 'Consolas');
    app.h.lbl_fov_v = uilabel(fov_panel, 'Position', [10 35 340 20], ...
        'Text', 'V: --', 'FontColor', [0.5 1 0.5], 'FontName', 'Consolas');
    app.h.lbl_fov_d = uilabel(fov_panel, 'Position', [10 15 340 20], ...
        'Text', 'Diagonal: --', 'FontColor', [0.5 1 0.5], 'FontName', 'Consolas');
    app.h.lbl_ifov = uilabel(fov_panel, 'Position', [200 55 160 20], ...
        'Text', 'IFOV: --', 'FontColor', [0.5 1 0.5], 'FontName', 'Consolas');
end

%% ========== Tab 2: Exposure/Gain ==========
function app = create_exposure_tab(app)
    tab = uitab(app.tabgroup, 'Title', 'Exposure', ...
        'BackgroundColor', [0.2 0.2 0.24], 'Scrollable', 'on');

    y = 820;

    % --- Exposure Time ---
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Exposure Time [ms]', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 25;
    app.h.lbl_exp = uilabel(tab, 'Position', [10 y 380 25], ...
        'Text', '22.0 ms', 'FontColor', [1 0.9 0.4], ...
        'FontName', 'Consolas', 'FontSize', 16, 'FontWeight', 'bold');
    y = y - 35;

    % 로그 스케일 슬라이더 (log2(1)=0 ~ log2(500)=~8.97)
    app.h.slider_exp = uislider(tab, 'Position', [10 y 360 3], ...
        'Limits', [0 log2(500)], 'Value', log2(app.params.exposure_ms));
    app.h.slider_exp.ValueChangedFcn = @(s,e) cb_exp_slider(s, e, app.fig);

    y = y - 30;
    uibutton(tab, 'Position', [10 y 55 26], 'Text', '1ms', ...
        'ButtonPushedFcn', @(s,e) cb_exp_preset(s, e, 1, app.fig));
    uibutton(tab, 'Position', [70 y 55 26], 'Text', '5ms', ...
        'ButtonPushedFcn', @(s,e) cb_exp_preset(s, e, 5, app.fig));
    uibutton(tab, 'Position', [130 y 55 26], 'Text', '22ms', ...
        'ButtonPushedFcn', @(s,e) cb_exp_preset(s, e, 22, app.fig));
    uibutton(tab, 'Position', [190 y 55 26], 'Text', '50ms', ...
        'ButtonPushedFcn', @(s,e) cb_exp_preset(s, e, 50, app.fig));
    uibutton(tab, 'Position', [250 y 55 26], 'Text', '100ms', ...
        'ButtonPushedFcn', @(s,e) cb_exp_preset(s, e, 100, app.fig));
    uibutton(tab, 'Position', [310 y 60 26], 'Text', '500ms', ...
        'ButtonPushedFcn', @(s,e) cb_exp_preset(s, e, 500, app.fig));

    % --- Analog Gain ---
    y = y - 50;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Analog Gain', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 25;
    app.h.lbl_gain = uilabel(tab, 'Position', [10 y 380 25], ...
        'Text', '16.0x', 'FontColor', [1 0.9 0.4], ...
        'FontName', 'Consolas', 'FontSize', 16, 'FontWeight', 'bold');
    y = y - 35;

    % 프리셋 버튼 (Python UI 스타일)
    gains = [1 2 4 8 16 32 64];
    for i = 1:length(gains)
        uibutton(tab, 'Position', [10+(i-1)*52 y 48 26], ...
            'Text', sprintf('%dx', gains(i)), ...
            'ButtonPushedFcn', @(s,e) cb_gain_preset(s, e, gains(i), app.fig));
    end

    y = y - 35;
    % 미세 조정 + 슬라이더
    uibutton(tab, 'Position', [10 y 45 26], 'Text', '-8', ...
        'ButtonPushedFcn', @(s,e) cb_gain_adjust(s, e, -8, app.fig));
    uibutton(tab, 'Position', [60 y 45 26], 'Text', '-1', ...
        'ButtonPushedFcn', @(s,e) cb_gain_adjust(s, e, -1, app.fig));
    app.h.slider_gain = uislider(tab, 'Position', [115 y 165 3], ...
        'Limits', [1 64], 'Value', app.params.analog_gain, ...
        'MajorTicks', [1 4 8 16 32 64]);
    app.h.slider_gain.ValueChangedFcn = @(s,e) cb_gain_slider(s, e, app.fig);
    uibutton(tab, 'Position', [295 y-10 45 26], 'Text', '+1', ...
        'ButtonPushedFcn', @(s,e) cb_gain_adjust(s, e, 1, app.fig));
    uibutton(tab, 'Position', [345 y-10 45 26], 'Text', '+8', ...
        'ButtonPushedFcn', @(s,e) cb_gain_adjust(s, e, 8, app.fig));

    % --- Digital Gain ---
    y = y - 60;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Digital Gain [x]', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 30;
    app.h.slider_dgain = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [0.5 4], 'Value', app.params.digital_gain, ...
        'MajorTicks', [0.5 1 2 3 4]);
    app.h.edit_dgain = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.digital_gain, 'Limits', [0.5 4], ...
        'ValueDisplayFormat', '%.1f');
    app.h.slider_dgain.ValueChangedFcn = @(s,e) cb_dgain(s, e, app.fig);
    app.h.edit_dgain.ValueChangedFcn = @(s,e) cb_dgain_edit(s, e, app.fig);

    % --- Signal Estimate ---
    y = y - 70;
    sig_panel = uipanel(tab, 'Position', [10 y-90 360 110], ...
        'Title', 'Signal Estimate (peak ADU)', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.15 0.15 0.2], 'ForegroundColor', [0.6 0.8 1]);
    app.h.lbl_sig1 = uilabel(sig_panel, 'Position', [10 65 340 20], ...
        'Text', '1st mag: --', 'FontColor', [0.6 0.8 1], 'FontName', 'Consolas');
    app.h.lbl_sig3 = uilabel(sig_panel, 'Position', [10 45 340 20], ...
        'Text', '3rd mag: --', 'FontColor', [0.6 0.8 1], 'FontName', 'Consolas');
    app.h.lbl_sig6 = uilabel(sig_panel, 'Position', [10 25 340 20], ...
        'Text', '6th mag: --', 'FontColor', [0.6 0.8 1], 'FontName', 'Consolas');
    app.h.lbl_sigsat = uilabel(sig_panel, 'Position', [10 5 340 20], ...
        'Text', 'Saturation: 255 ADU', 'FontColor', [1 0.5 0.5], 'FontName', 'Consolas');
end

%% ========== Tab 3: Noise ==========
function app = create_noise_tab(app)
    tab = uitab(app.tabgroup, 'Title', 'Noise', ...
        'BackgroundColor', [0.2 0.2 0.24], 'Scrollable', 'on');

    y = 820;

    % --- Noise Enable ---
    app.h.chk_noise = uicheckbox(tab, 'Position', [10 y 200 22], ...
        'Text', 'Enable Noise', 'Value', app.params.noise_enabled, ...
        'FontColor', 'w', 'FontSize', 14, 'FontWeight', 'bold');
    app.h.chk_noise.ValueChangedFcn = @(s,e) cb_noise_toggle(s, e, app.fig);

    % --- Dark Current ---
    y = y - 50;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Dark Current [e-/px/s]', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 30;
    app.h.slider_dark = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [0 1], 'Value', app.params.dark_current, ...
        'MajorTicks', 0:0.2:1);
    app.h.edit_dark = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.dark_current, 'Limits', [0 1], ...
        'ValueDisplayFormat', '%.3f');
    app.h.slider_dark.ValueChangedFcn = @(s,e) cb_noise_param(s, e, 'dark_current', app.fig);
    app.h.edit_dark.ValueChangedFcn = @(s,e) cb_noise_param_edit(s, e, 'dark_current', app.fig);

    y = y - 30;
    uibutton(tab, 'Position', [10 y 100 26], 'Text', 'Space (0.01)', ...
        'ButtonPushedFcn', @(s,e) cb_dark_preset(s, e, 0.01, app.fig));
    uibutton(tab, 'Position', [115 y 100 26], 'Text', 'Ground (0.1)', ...
        'ButtonPushedFcn', @(s,e) cb_dark_preset(s, e, 0.1, app.fig));

    % --- Read Noise ---
    y = y - 50;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Read Noise [e- RMS]', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 30;
    app.h.slider_read = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [0 20], 'Value', app.params.read_noise, ...
        'MajorTicks', 0:5:20);
    app.h.edit_read = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.read_noise, 'Limits', [0 20], ...
        'ValueDisplayFormat', '%.1f');
    app.h.slider_read.ValueChangedFcn = @(s,e) cb_noise_param(s, e, 'read_noise', app.fig);
    app.h.edit_read.ValueChangedFcn = @(s,e) cb_noise_param_edit(s, e, 'read_noise', app.fig);

    % --- Noise Budget ---
    y = y - 70;
    noise_panel = uipanel(tab, 'Position', [10 y-90 360 110], ...
        'Title', 'Noise Budget', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.15 0.15], 'ForegroundColor', [1 0.7 0.7]);
    app.h.lbl_nshot = uilabel(noise_panel, 'Position', [10 65 340 20], ...
        'Text', 'Shot noise (6mag): --', 'FontColor', [1 0.7 0.7], 'FontName', 'Consolas');
    app.h.lbl_nread = uilabel(noise_panel, 'Position', [10 45 340 20], ...
        'Text', 'Read noise (ADU): --', 'FontColor', [1 0.7 0.7], 'FontName', 'Consolas');
    app.h.lbl_ndark = uilabel(noise_panel, 'Position', [10 25 340 20], ...
        'Text', 'Dark current (ADU): --', 'FontColor', [1 0.7 0.7], 'FontName', 'Consolas');
    app.h.lbl_ntotal = uilabel(noise_panel, 'Position', [10 5 340 20], ...
        'Text', 'Total noise RMS: --', 'FontColor', [1 0.5 0.5], ...
        'FontName', 'Consolas', 'FontWeight', 'bold');

    % --- Scene Presets ---
    y = y - 130;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Scene Presets', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 30;
    uibutton(tab, 'Position', [10 y 85 28], 'Text', 'Space', ...
        'BackgroundColor', [0.2 0.3 0.5], ...
        'ButtonPushedFcn', @(s,e) cb_scene_preset(s, e, 'space', app.fig));
    uibutton(tab, 'Position', [100 y 85 28], 'Text', 'Ground', ...
        'BackgroundColor', [0.3 0.3 0.2], ...
        'ButtonPushedFcn', @(s,e) cb_scene_preset(s, e, 'ground', app.fig));
    uibutton(tab, 'Position', [190 y 85 28], 'Text', 'Low Noise', ...
        'BackgroundColor', [0.2 0.35 0.2], ...
        'ButtonPushedFcn', @(s,e) cb_scene_preset(s, e, 'low_noise', app.fig));
    uibutton(tab, 'Position', [280 y 85 28], 'Text', 'High Noise', ...
        'BackgroundColor', [0.4 0.2 0.2], ...
        'ButtonPushedFcn', @(s,e) cb_scene_preset(s, e, 'high_noise', app.fig));
end

%% ========== Tab 4: Observation ==========
function app = create_observation_tab(app)
    tab = uitab(app.tabgroup, 'Title', 'Observation', ...
        'BackgroundColor', [0.2 0.2 0.24], 'Scrollable', 'on');

    y = 820;

    % --- RA ---
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Right Ascension (RA) [deg]', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 30;
    app.h.slider_ra = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [0 360], 'Value', app.params.ra, 'MajorTicks', 0:60:360);
    app.h.edit_ra = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.ra, 'Limits', [0 360], ...
        'ValueDisplayFormat', '%.2f');
    app.h.slider_ra.ValueChangedFcn = @(s,e) cb_obs(s, e, 'ra', app.fig);
    app.h.edit_ra.ValueChangedFcn = @(s,e) cb_obs_edit(s, e, 'ra', app.fig);

    % --- DEC ---
    y = y - 55;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Declination (DEC) [deg]', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 30;
    app.h.slider_dec = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [-90 90], 'Value', app.params.dec, 'MajorTicks', -90:30:90);
    app.h.edit_dec = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.dec, 'Limits', [-90 90], ...
        'ValueDisplayFormat', '%.2f');
    app.h.slider_dec.ValueChangedFcn = @(s,e) cb_obs(s, e, 'dec', app.fig);
    app.h.edit_dec.ValueChangedFcn = @(s,e) cb_obs_edit(s, e, 'dec', app.fig);

    % --- Roll ---
    y = y - 55;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Roll [deg]', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 30;
    app.h.slider_roll = uislider(tab, 'Position', [10 y 260 3], ...
        'Limits', [0 360], 'Value', app.params.roll, 'MajorTicks', 0:45:360);
    app.h.edit_roll = uieditfield(tab, 'numeric', 'Position', [290 y-10 70 22], ...
        'Value', app.params.roll, 'Limits', [0 360], ...
        'ValueDisplayFormat', '%.1f');
    app.h.slider_roll.ValueChangedFcn = @(s,e) cb_obs(s, e, 'roll', app.fig);
    app.h.edit_roll.ValueChangedFcn = @(s,e) cb_obs_edit(s, e, 'roll', app.fig);

    % --- Observation Presets ---
    y = y - 60;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Target Presets', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 35;
    presets = {
        'Orion Belt', 84, -1;
        'Polaris', 37.95, 89.26;
        'Big Dipper', 165, 55;
        'Antares', 247.35, -26.43;
        'Random', -1, -1;
    };
    for i = 1:size(presets, 1)
        uibutton(tab, 'Position', [10 y-(i-1)*32 360 28], ...
            'Text', sprintf('%s  (RA=%.1f, DEC=%.1f)', presets{i,1}, presets{i,2}, presets{i,3}), ...
            'ButtonPushedFcn', @(s,e) cb_obs_preset(s, e, presets{i,2}, presets{i,3}, app.fig));
    end
end

%% ========== Tab 5: Motion ==========
function app = create_motion_tab(app)
    tab = uitab(app.tabgroup, 'Title', 'Motion', ...
        'BackgroundColor', [0.2 0.2 0.24], 'Scrollable', 'on');

    y = 820;

    % --- Angular Velocity ---
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Angular Velocity [deg/s]', ...
        'FontColor', [1 0.7 0.3], 'FontWeight', 'bold', 'FontSize', 13);

    % X axis
    y = y - 35;
    uilabel(tab, 'Position', [10 y 80 20], 'Text', 'Pitch (X):', ...
        'FontColor', [1 0.5 0.5], 'FontWeight', 'bold');
    app.h.slider_ox = uislider(tab, 'Position', [10 y-25 260 3], ...
        'Limits', [-10 10], 'Value', app.params.omega_x, 'MajorTicks', -10:2:10);
    app.h.edit_ox = uieditfield(tab, 'numeric', 'Position', [290 y-35 70 22], ...
        'Value', app.params.omega_x, 'Limits', [-10 10], ...
        'ValueDisplayFormat', '%.2f');
    app.h.slider_ox.ValueChangedFcn = @(s,e) cb_motion(s, e, 'omega_x', app.fig);
    app.h.edit_ox.ValueChangedFcn = @(s,e) cb_motion_edit(s, e, 'omega_x', app.fig);

    % Y axis
    y = y - 60;
    uilabel(tab, 'Position', [10 y 80 20], 'Text', 'Yaw (Y):', ...
        'FontColor', [0.5 1 0.5], 'FontWeight', 'bold');
    app.h.slider_oy = uislider(tab, 'Position', [10 y-25 260 3], ...
        'Limits', [-10 10], 'Value', app.params.omega_y, 'MajorTicks', -10:2:10);
    app.h.edit_oy = uieditfield(tab, 'numeric', 'Position', [290 y-35 70 22], ...
        'Value', app.params.omega_y, 'Limits', [-10 10], ...
        'ValueDisplayFormat', '%.2f');
    app.h.slider_oy.ValueChangedFcn = @(s,e) cb_motion(s, e, 'omega_y', app.fig);
    app.h.edit_oy.ValueChangedFcn = @(s,e) cb_motion_edit(s, e, 'omega_y', app.fig);

    % Z axis
    y = y - 60;
    uilabel(tab, 'Position', [10 y 80 20], 'Text', 'Roll (Z):', ...
        'FontColor', [0.5 0.7 1], 'FontWeight', 'bold');
    app.h.slider_oz = uislider(tab, 'Position', [10 y-25 260 3], ...
        'Limits', [-10 10], 'Value', app.params.omega_z, 'MajorTicks', -10:2:10);
    app.h.edit_oz = uieditfield(tab, 'numeric', 'Position', [290 y-35 70 22], ...
        'Value', app.params.omega_z, 'Limits', [-10 10], ...
        'ValueDisplayFormat', '%.2f');
    app.h.slider_oz.ValueChangedFcn = @(s,e) cb_motion(s, e, 'omega_z', app.fig);
    app.h.edit_oz.ValueChangedFcn = @(s,e) cb_motion_edit(s, e, 'omega_z', app.fig);

    % --- Preset buttons ---
    y = y - 55;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Motion Presets', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 30;
    uibutton(tab, 'Position', [10 y 85 26], 'Text', 'Static', ...
        'ButtonPushedFcn', @(s,e) cb_motion_preset(s, e, [0 0 0], app.fig));
    uibutton(tab, 'Position', [100 y 85 26], 'Text', '0.5 deg/s', ...
        'ButtonPushedFcn', @(s,e) cb_motion_preset(s, e, [0 0.5 0], app.fig));
    uibutton(tab, 'Position', [190 y 85 26], 'Text', '2 deg/s', ...
        'ButtonPushedFcn', @(s,e) cb_motion_preset(s, e, [0 2 0], app.fig));
    uibutton(tab, 'Position', [280 y 85 26], 'Text', '5 deg/s', ...
        'ButtonPushedFcn', @(s,e) cb_motion_preset(s, e, [0 5 0], app.fig));

    % --- Motion Info Panel ---
    y = y - 50;
    motion_panel = uipanel(tab, 'Position', [10 y-120 360 140], ...
        'Title', 'Motion Effect (computed)', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.18 0.15 0.12], 'ForegroundColor', [1 0.8 0.5]);
    app.h.lbl_mot_shutter = uilabel(motion_panel, 'Position', [10 95 340 20], ...
        'Text', 'Shutter: Global', 'FontColor', [0.6 0.8 1], 'FontName', 'Consolas');
    app.h.lbl_mot_blur = uilabel(motion_panel, 'Position', [10 75 340 20], ...
        'Text', 'Motion blur: 0.0 px', 'FontColor', [1 0.8 0.5], 'FontName', 'Consolas');
    app.h.lbl_mot_rs = uilabel(motion_panel, 'Position', [10 55 340 20], ...
        'Text', 'RS max shift: 0.0 px', 'FontColor', [1 0.7 0.7], 'FontName', 'Consolas');
    app.h.lbl_mot_adc = uilabel(motion_panel, 'Position', [10 35 340 20], ...
        'Text', 'ADC: 8-bit (0-255)', 'FontColor', [0.7 0.7 0.7], 'FontName', 'Consolas');
    app.h.lbl_mot_psf = uilabel(motion_panel, 'Position', [10 15 340 20], ...
        'Text', 'PSF sigma: 1.20 px', 'FontColor', [0.7 0.7 0.7], 'FontName', 'Consolas');

    % --- Environment ---
    y = y - 155;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Environment', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 28;
    app.h.dd_env = uidropdown(tab, 'Position', [10 y 170 24], ...
        'Items', {'space', 'ground'}, 'Value', app.params.environment);
    app.h.dd_env.ValueChangedFcn = @(s,e) cb_env_changed(s, e, app.fig);

    uilabel(tab, 'Position', [190 y 80 22], 'Text', 'Temp [C]:', ...
        'FontColor', [0.7 0.7 0.7]);
    app.h.edit_temp = uieditfield(tab, 'numeric', 'Position', [270 y 90 22], ...
        'Value', app.params.sensor_temp, 'Limits', [-60 60], ...
        'ValueDisplayFormat', '%.0f');
    app.h.edit_temp.ValueChangedFcn = @(s,e) cb_temp_changed(s, e, app.fig);
end

%% ========== Tab 6: Processing ==========
function app = create_processing_tab(app)
    tab = uitab(app.tabgroup, 'Title', 'Processing', ...
        'BackgroundColor', [0.2 0.2 0.24], 'Scrollable', 'on');

    y = 820;

    % --- Conversion Method ---
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Bayer -> Gray Method', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 30;
    app.h.dd_method = uidropdown(tab, 'Position', [10 y 360 28], ...
        'Items', {'raw', 'binning', 'green', 'weighted', 'optimal'}, ...
        'Value', app.params.conv_method);
    app.h.dd_method.ValueChangedFcn = @(s,e) cb_method_changed(s, e, app.fig);

    % --- Optimal Weights ---
    y = y - 45;
    app.h.panel_weights = uipanel(tab, 'Position', [10 y-130 360 160], ...
        'Title', 'Optimal Weights (SNR Maximized)', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.18 0.22], 'ForegroundColor', [1 0.8 1]);

    wp = app.h.panel_weights;
    uilabel(wp, 'Position', [10 115 30 20], 'Text', 'R:', ...
        'FontColor', [1 0.4 0.4], 'FontWeight', 'bold');
    app.h.slider_wR = uislider(wp, 'Position', [35 115 220 3], ...
        'Limits', [0 1], 'Value', app.params.opt_weights(1));
    app.h.lbl_wR = uilabel(wp, 'Position', [270 105 80 20], ...
        'Text', sprintf('%.4f', app.params.opt_weights(1)), ...
        'FontColor', [1 0.4 0.4], 'FontName', 'Consolas');

    uilabel(wp, 'Position', [10 80 30 20], 'Text', 'G:', ...
        'FontColor', [0.4 1 0.4], 'FontWeight', 'bold');
    app.h.slider_wG = uislider(wp, 'Position', [35 80 220 3], ...
        'Limits', [0 1], 'Value', app.params.opt_weights(2));
    app.h.lbl_wG = uilabel(wp, 'Position', [270 70 80 20], ...
        'Text', sprintf('%.4f', app.params.opt_weights(2)), ...
        'FontColor', [0.4 1 0.4], 'FontName', 'Consolas');

    uilabel(wp, 'Position', [10 45 30 20], 'Text', 'B:', ...
        'FontColor', [0.4 0.6 1], 'FontWeight', 'bold');
    app.h.slider_wB = uislider(wp, 'Position', [35 45 220 3], ...
        'Limits', [0 1], 'Value', app.params.opt_weights(3));
    app.h.lbl_wB = uilabel(wp, 'Position', [270 35 80 20], ...
        'Text', sprintf('%.4f', app.params.opt_weights(3)), ...
        'FontColor', [0.4 0.6 1], 'FontName', 'Consolas');

    app.h.lbl_wsum = uilabel(wp, 'Position', [10 10 100 20], ...
        'Text', 'Sum: 1.000', 'FontColor', 'w', 'FontName', 'Consolas');
    uibutton(wp, 'Position', [130 5 100 22], 'Text', 'Normalize', ...
        'ButtonPushedFcn', @(s,e) cb_normalize_weights(s, e, app.fig));
    uibutton(wp, 'Position', [240 5 100 22], 'Text', 'Reset', ...
        'ButtonPushedFcn', @(s,e) cb_reset_weights(s, e, app.fig));

    app.h.slider_wR.ValueChangedFcn = @(s,e) cb_weight_changed(s, e, app.fig);
    app.h.slider_wG.ValueChangedFcn = @(s,e) cb_weight_changed(s, e, app.fig);
    app.h.slider_wB.ValueChangedFcn = @(s,e) cb_weight_changed(s, e, app.fig);

    % optimal이 아닌 경우 패널 비활성화 표시
    app.h.panel_weights.Visible = strcmp(app.params.conv_method, 'optimal');

    % --- Star Detection ---
    y = y - 195;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Star Detection', ...
        'FontColor', 'w', 'FontWeight', 'bold');

    y = y - 25;
    uilabel(tab, 'Position', [10 y 150 20], 'Text', 'Threshold [ADU]:', ...
        'FontColor', [0.7 0.7 0.7]);
    app.h.slider_thresh = uislider(tab, 'Position', [10 y-25 260 3], ...
        'Limits', [1 100], 'Value', app.params.threshold, ...
        'MajorTicks', [1 15 30 50 75 100]);
    app.h.edit_thresh = uieditfield(tab, 'numeric', 'Position', [290 y-35 70 22], ...
        'Value', app.params.threshold, 'Limits', [1 100], ...
        'RoundFractionalValues', 'on');
    app.h.slider_thresh.ValueChangedFcn = @(s,e) cb_detect_param(s, e, 'threshold', app.fig);
    app.h.edit_thresh.ValueChangedFcn = @(s,e) cb_detect_param_edit(s, e, 'threshold', app.fig);

    y = y - 65;
    uilabel(tab, 'Position', [10 y 150 20], 'Text', 'Min Area [px]:', ...
        'FontColor', [0.7 0.7 0.7]);
    app.h.slider_minarea = uislider(tab, 'Position', [10 y-25 260 3], ...
        'Limits', [1 20], 'Value', app.params.min_area, ...
        'MajorTicks', [1 2 5 10 15 20]);
    app.h.edit_minarea = uieditfield(tab, 'numeric', 'Position', [290 y-35 70 22], ...
        'Value', app.params.min_area, 'Limits', [1 20], ...
        'RoundFractionalValues', 'on');
    app.h.slider_minarea.ValueChangedFcn = @(s,e) cb_detect_param(s, e, 'min_area', app.fig);
    app.h.edit_minarea.ValueChangedFcn = @(s,e) cb_detect_param_edit(s, e, 'min_area', app.fig);

    % --- Display Options ---
    y = y - 65;
    uilabel(tab, 'Position', [10 y 380 20], 'Text', 'Display Options', ...
        'FontColor', 'w', 'FontWeight', 'bold');
    y = y - 28;
    app.h.chk_overlay = uicheckbox(tab, 'Position', [10 y 200 22], ...
        'Text', 'Show Detection Overlay', 'Value', false, 'FontColor', [0.7 0.7 0.7]);
    app.h.chk_overlay.ValueChangedFcn = @(s,e) cb_display_opt(s, e, app.fig);
    y = y - 28;
    app.h.chk_labels = uicheckbox(tab, 'Position', [10 y 200 22], ...
        'Text', 'Show Magnitude Labels', 'Value', false, 'FontColor', [0.7 0.7 0.7]);
    app.h.chk_labels.ValueChangedFcn = @(s,e) cb_display_opt(s, e, app.fig);
end

%% ========== Action Buttons ==========
function app = create_action_buttons(app)
    fw = app.fig_w;
    fh = app.fig_h;
    btn_y = fh - 38;
    x0 = 415;

    app.h.btn_sim = uibutton(app.fig, 'Position', [x0 btn_y 140 30], ...
        'Text', 'Simulate', 'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.6 0.3], 'FontColor', 'w');
    app.h.btn_sim.ButtonPushedFcn = @(s,e) cb_simulate(s, e, app.fig);

    app.h.btn_quick = uibutton(app.fig, 'Position', [x0+150 btn_y 140 30], ...
        'Text', 'Quick Update', 'FontSize', 12, ...
        'BackgroundColor', [0.2 0.4 0.6], 'FontColor', 'w');
    app.h.btn_quick.ButtonPushedFcn = @(s,e) cb_quick_update(s, e, app.fig);

    app.h.btn_export = uibutton(app.fig, 'Position', [x0+300 btn_y 120 30], ...
        'Text', 'Export', 'FontSize', 11);
    app.h.btn_export.ButtonPushedFcn = @(s,e) cb_export(s, e, app.fig);

    % 상태 표시
    status_x = x0 + 435;
    status_w = max(200, fw - status_x - 10);
    app.h.lbl_status = uilabel(app.fig, 'Position', [status_x btn_y status_w 30], ...
        'Text', 'Ready - Press [Simulate] to generate star images', ...
        'FontColor', [0.6 0.8 1], 'FontSize', 12, 'FontName', 'Consolas');
end

%% ========== Display Controls Bar ==========
function app = create_display_controls(app)
    fw = app.fig_w;
    fh = app.fig_h;
    x0 = 415;
    bar_y = fh - 73;    % 버튼 바로 아래
    cx = x0;             % 현재 x 위치

    % "Display:" 라벨
    uilabel(app.fig, 'Position', [cx bar_y 55 25], ...
        'Text', 'Display:', 'FontColor', [0.7 0.8 1], ...
        'FontSize', 11, 'FontWeight', 'bold');
    cx = cx + 58;

    % 모드 드롭다운
    app.h.dd_display_mode = uidropdown(app.fig, ...
        'Position', [cx bar_y 120 25], ...
        'Items', {'RAW', 'Digital Gain', 'Window', 'Gamma'}, ...
        'Value', 'RAW', 'FontSize', 10);
    app.h.dd_display_mode.ValueChangedFcn = @(s,e) cb_display_mode(s, e, app.fig);
    cx = cx + 128;

    % === Digital Gain 컨트롤 (모드별 표시) ===
    app.h.dd_dgain = uidropdown(app.fig, ...
        'Position', [cx bar_y 70 25], ...
        'Items', {'x1', 'x2', 'x4', 'x8', 'x16'}, ...
        'Value', 'x1', 'FontSize', 10, 'Visible', 'off');
    app.h.dd_dgain.ValueChangedFcn = @(s,e) cb_display_dgain(s, e, app.fig);

    % === Window 컨트롤 (Black / White 슬라이더) ===
    app.h.lbl_win_black = uilabel(app.fig, 'Position', [cx bar_y 25 25], ...
        'Text', 'B:', 'FontColor', [0.6 0.6 0.7], 'FontSize', 10, 'Visible', 'off');
    app.h.sld_win_black = uislider(app.fig, 'Position', [cx+25 bar_y+13 120 3], ...
        'Limits', [0 4095], 'Value', 0, 'Visible', 'off');
    app.h.sld_win_black.ValueChangedFcn = @(s,e) cb_display_window(s, e, app.fig);
    app.h.lbl_win_white = uilabel(app.fig, 'Position', [cx+155 bar_y 25 25], ...
        'Text', 'W:', 'FontColor', [0.6 0.6 0.7], 'FontSize', 10, 'Visible', 'off');
    app.h.sld_win_white = uislider(app.fig, 'Position', [cx+180 bar_y+13 120 3], ...
        'Limits', [0 4095], 'Value', 4095, 'Visible', 'off');
    app.h.sld_win_white.ValueChangedFcn = @(s,e) cb_display_window(s, e, app.fig);
    app.h.lbl_win_val = uilabel(app.fig, 'Position', [cx+305 bar_y 120 25], ...
        'Text', '[0 - 4095]', 'FontColor', [0.5 0.5 0.6], ...
        'FontSize', 9, 'FontName', 'Consolas', 'Visible', 'off');

    % === Gamma 컨트롤 ===
    app.h.lbl_gamma = uilabel(app.fig, 'Position', [cx bar_y 15 25], ...
        'Text', 'γ:', 'FontColor', [0.6 0.6 0.7], 'FontSize', 10, 'Visible', 'off');
    app.h.sld_gamma = uislider(app.fig, 'Position', [cx+18 bar_y+13 180 3], ...
        'Limits', [0.1 1.0], 'Value', 0.5, 'Visible', 'off');
    app.h.sld_gamma.ValueChangedFcn = @(s,e) cb_display_gamma(s, e, app.fig);
    app.h.lbl_gamma_val = uilabel(app.fig, 'Position', [cx+205 bar_y 60 25], ...
        'Text', '0.50', 'FontColor', [0.5 0.5 0.6], ...
        'FontSize', 10, 'FontName', 'Consolas', 'Visible', 'off');

    % === 히스토그램 토글 버튼 (우측 끝) ===
    hist_x = fw - 120;
    app.h.btn_histogram = uibutton(app.fig, 'Position', [hist_x bar_y 100 25], ...
        'Text', 'Histogram', 'FontSize', 10, ...
        'BackgroundColor', [0.25 0.25 0.3], 'FontColor', [0.7 0.7 0.8]);
    app.h.btn_histogram.ButtonPushedFcn = @(s,e) cb_histogram_toggle(s, e, app.fig);
end

%% ========== Image Area ==========
function app = create_image_area(app)
    fw = app.fig_w;
    fh = app.fig_h;
    x0 = 415;
    right_w = fw - x0 - 10;          % 우측 가용 너비
    metrics_h = 210;                   % 하단 메트릭 높이
    btn_h = 38;                        % 상단 버튼 높이
    disp_bar_h = 35;                   % 디스플레이 컨트롤 바 높이
    gap = 15;
    hist_h = 0;                        % 히스토그램 높이 (초기: 0, ON 시 120)

    img_y = metrics_h + 10 + hist_h;
    img_h = fh - btn_h - disp_bar_h - metrics_h - 50 - hist_h;
    img_w = round((right_w - gap) / 2);

    % 레이아웃 파라미터 저장 (동적 리사이즈용)
    app.layout.x0 = x0;
    app.layout.right_w = right_w;
    app.layout.metrics_h = metrics_h;
    app.layout.btn_h = btn_h;
    app.layout.disp_bar_h = disp_bar_h;
    app.layout.gap = gap;
    app.layout.img_w = img_w;

    % 좌: Ideal Grayscale
    app.h.lbl_ideal = uilabel(app.fig, 'Position', [x0 img_y+img_h img_w 22], ...
        'Text', 'Ideal Grayscale (No Noise)', ...
        'FontColor', [0.5 1 0.5], 'FontSize', 12, 'FontWeight', 'bold');
    app.h.ax_ideal = uiaxes(app.fig, 'Position', [x0 img_y img_w img_h]);
    app.h.ax_ideal.Color = [0.05 0.05 0.08];
    app.h.ax_ideal.XTick = [];
    app.h.ax_ideal.YTick = [];
    app.h.ax_ideal.Box = 'on';
    app.h.ax_ideal.XColor = [0.3 0.3 0.3];
    app.h.ax_ideal.YColor = [0.3 0.3 0.3];
    title(app.h.ax_ideal, 'No simulation yet', 'Color', [0.5 0.5 0.5]);

    % 우: Converted Grayscale
    x_right = x0 + img_w + gap;
    app.h.lbl_conv = uilabel(app.fig, 'Position', [x_right img_y+img_h img_w 22], ...
        'Text', 'Converted Grayscale (Bayer -> Gray)', ...
        'FontColor', [1 0.8 0.4], 'FontSize', 12, 'FontWeight', 'bold');
    app.h.ax_conv = uiaxes(app.fig, 'Position', [x_right img_y img_w img_h]);
    app.h.ax_conv.Color = [0.05 0.05 0.08];
    app.h.ax_conv.XTick = [];
    app.h.ax_conv.YTick = [];
    app.h.ax_conv.Box = 'on';
    app.h.ax_conv.XColor = [0.3 0.3 0.3];
    app.h.ax_conv.YColor = [0.3 0.3 0.3];
    title(app.h.ax_conv, 'No simulation yet', 'Color', [0.5 0.5 0.5]);

    % 히스토그램 axes (초기: 숨김)
    hist_area_h = 120;
    hist_y = metrics_h + 10;
    app.h.ax_hist_ideal = uiaxes(app.fig, 'Position', [x0 hist_y img_w hist_area_h], ...
        'Visible', 'off');
    app.h.ax_hist_ideal.Color = [0.08 0.08 0.1];
    app.h.ax_hist_ideal.XTick = [];
    app.h.ax_hist_ideal.YTick = [];
    app.h.ax_hist_ideal.Box = 'on';
    app.h.ax_hist_ideal.XColor = [0.4 0.4 0.5];
    app.h.ax_hist_ideal.YColor = [0.4 0.4 0.5];

    app.h.ax_hist_conv = uiaxes(app.fig, 'Position', [x_right hist_y img_w hist_area_h], ...
        'Visible', 'off');
    app.h.ax_hist_conv.Color = [0.08 0.08 0.1];
    app.h.ax_hist_conv.XTick = [];
    app.h.ax_hist_conv.YTick = [];
    app.h.ax_hist_conv.Box = 'on';
    app.h.ax_hist_conv.XColor = [0.4 0.4 0.5];
    app.h.ax_hist_conv.YColor = [0.4 0.4 0.5];
end

%% ========== Metrics Panel ==========
function app = create_metrics_panel(app)
    fw = app.fig_w;
    panel_w = fw - 415;
    mp = uipanel(app.fig, 'Position', [415 5 panel_w 210], ...
        'BackgroundColor', [0.12 0.12 0.16], 'BorderType', 'line', ...
        'HighlightColor', [0.3 0.3 0.4]);

    % 3열 균등 분할
    cw = round(panel_w / 3);
    col1 = 15;
    col2 = cw;
    col3 = cw * 2;
    label_w = cw - 20;

    % 좌측 열: FOV + 센서 정보
    app.h.lbl_m_fov = uilabel(mp, 'Position', [col1 170 label_w 20], ...
        'Text', 'FOV: -- x -- deg', 'FontColor', [0.5 1 0.5], ...
        'FontName', 'Consolas', 'FontSize', 12);
    app.h.lbl_m_res = uilabel(mp, 'Position', [col1 148 label_w 20], ...
        'Text', 'Resolution: --', 'FontColor', [0.7 0.7 0.7], ...
        'FontName', 'Consolas', 'FontSize', 11);
    app.h.lbl_m_ifov = uilabel(mp, 'Position', [col1 126 label_w 20], ...
        'Text', 'IFOV: -- deg/px', 'FontColor', [0.7 0.7 0.7], ...
        'FontName', 'Consolas', 'FontSize', 11);
    app.h.lbl_m_pointing = uilabel(mp, 'Position', [col1 104 label_w 20], ...
        'Text', 'Pointing: RA=--, DEC=--', 'FontColor', [0.7 0.7 0.7], ...
        'FontName', 'Consolas', 'FontSize', 11);

    % 중앙 열: 별 정보
    app.h.lbl_m_stars = uilabel(mp, 'Position', [col2 170 label_w 20], ...
        'Text', 'Stars in FOV: --', 'FontColor', [1 1 0.5], ...
        'FontName', 'Consolas', 'FontSize', 12, 'FontWeight', 'bold');
    app.h.lbl_m_detected = uilabel(mp, 'Position', [col2 148 label_w 20], ...
        'Text', 'Detected: -- (--)', 'FontColor', [0.7 0.7 0.7], ...
        'FontName', 'Consolas', 'FontSize', 11);
    app.h.lbl_m_snr = uilabel(mp, 'Position', [col2 126 label_w 20], ...
        'Text', 'Peak SNR: -- dB', 'FontColor', [0.7 0.7 0.7], ...
        'FontName', 'Consolas', 'FontSize', 11);
    app.h.lbl_m_rms = uilabel(mp, 'Position', [col2 104 label_w 20], ...
        'Text', 'Centroid RMS: -- px', 'FontColor', [0.7 0.7 0.7], ...
        'FontName', 'Consolas', 'FontSize', 11);

    % 우측 열: 처리 정보
    app.h.lbl_m_method = uilabel(mp, 'Position', [col3 170 label_w 20], ...
        'Text', 'Method: --', 'FontColor', [1 0.8 0.4], ...
        'FontName', 'Consolas', 'FontSize', 12, 'FontWeight', 'bold');
    app.h.lbl_m_simtime = uilabel(mp, 'Position', [col3 148 label_w 20], ...
        'Text', 'Sim time: --', 'FontColor', [0.7 0.7 0.7], ...
        'FontName', 'Consolas', 'FontSize', 11);
    app.h.lbl_m_convtime = uilabel(mp, 'Position', [col3 126 label_w 20], ...
        'Text', 'Conv time: --', 'FontColor', [0.7 0.7 0.7], ...
        'FontName', 'Consolas', 'FontSize', 11);
    app.h.lbl_m_exposure = uilabel(mp, 'Position', [col3 104 label_w 20], ...
        'Text', 'Exp: -- | Gain: --', 'FontColor', [0.7 0.7 0.7], ...
        'FontName', 'Consolas', 'FontSize', 11);

    % 하단: 밝은 별 목록 (간략)
    app.h.lbl_m_brightest = uilabel(mp, 'Position', [col1 5 panel_w-30 90], ...
        'Text', 'Brightest stars: (run simulation first)', ...
        'FontColor', [0.5 0.5 0.6], 'FontName', 'Consolas', 'FontSize', 10, ...
        'VerticalAlignment', 'top');
end

%% ========== build_sensor_params: GUI -> struct ==========
function sp = build_sensor_params(app)
    p = app.params;
    sp = struct();
    sp.myu = p.pixel_size * 1e-6;        % um -> m
    sp.f = p.focal_length * 1e-3;         % mm -> m
    % Preview scale 적용 (해상도만 줄이고 FOV 유지 → 픽셀 크기 스케일)
    scale = p.preview_scale;
    if scale > 1
        sp.l = round(p.res_w / scale);
        sp.w = round(p.res_h / scale);
        sp.myu = sp.myu * scale;  % 픽셀 크기 확대 → FOV 유지
        sp.psf_sigma = p.psf_sigma / scale;  % PSF도 축소
    else
        sp.l = p.res_w;
        sp.w = p.res_h;
    end
    sp.mag_limit = p.mag_limit;
    sp.exposure_time = p.exposure_ms * 1e-3;  % ms -> s
    sp.analog_gain = p.analog_gain;
    sp.digital_gain = p.digital_gain;
    sp.quantum_efficiency = p.qe;
    sp.dark_current_ref = p.dark_current;
    sp.read_noise = p.read_noise;
    sp.add_noise = p.noise_enabled;
    sp.sensitivity_R = p.sens_R;
    sp.sensitivity_G = p.sens_G;
    sp.sensitivity_B = p.sens_B;

    % Advanced sensor params
    sp.shutter_type = p.shutter_type;
    sp.readout_time = p.readout_time;
    sp.angular_velocity = [p.omega_x, p.omega_y, p.omega_z];
    sp.adc_bits = p.adc_bits;
    sp.full_well = p.full_well;
    if scale <= 1
        sp.psf_sigma = p.psf_sigma;
    end
    sp.environment = p.environment;
    sp.sensor_temp = p.sensor_temp;

    % 카탈로그 사전 로드
    if ~isempty(app.catalog)
        sp.preloaded_catalog = app.catalog;
    end
end

%% ========== 시뮬레이션 파이프라인 ==========
function run_simulation(fig, force_stage1)
    app = fig.UserData;
    if nargin < 2, force_stage1 = true; end

    % Stage 1: Star image generation
    if app.dirty.stage1 || force_stage1
        app.h.lbl_status.Text = 'Simulating stars...';
        drawnow;

        sp = build_sensor_params(app);
        tic;
        [~, bayer_img, star_info] = ...
            simulate_star_image_realistic(app.params.ra, app.params.dec, ...
            app.params.roll, sp);
        sim_time = toc;

        % 노이즈 없는 이상적 이미지 사용 (star_info.ideal_gray)
        app.cache.gray_ideal = star_info.ideal_gray;
        app.cache.bayer_img = bayer_img;
        app.cache.star_info = star_info;
        app.cache.sim_time = sim_time;
        app.dirty.stage1 = false;
        app.dirty.stage2 = true;
        app.dirty.stage3 = true;
    end

    % Stage 2: Bayer -> Gray conversion
    if app.dirty.stage2
        app.h.lbl_status.Text = 'Converting Bayer -> Gray...';
        drawnow;

        method = app.params.conv_method;
        tic;
        if strcmp(method, 'optimal')
            [gray_conv, method_info] = bayer_to_gray_direct(...
                app.cache.bayer_img, method, app.params.opt_weights);
        else
            [gray_conv, method_info] = bayer_to_gray_direct(...
                app.cache.bayer_img, method);
        end
        conv_time = toc;

        app.cache.gray_converted = gray_conv;
        app.cache.method_info = method_info;
        app.cache.conv_time = conv_time;
        app.dirty.stage2 = false;
        app.dirty.stage3 = true;
    end

    % Stage 3: Detection + Metrics
    if app.dirty.stage3
        app.h.lbl_status.Text = 'Detecting stars...';
        drawnow;

        gray = app.cache.gray_converted;
        threshold = round(app.params.threshold);
        min_area = round(app.params.min_area);

        % 별 검출 (result struct 반환)
        det_result = detect_stars_simple(gray, threshold, min_area);
        app.cache.detection = det_result;

        % SNR
        app.cache.snr_db = calculate_peak_snr(gray);

        % Centroid accuracy
        if app.cache.star_info.num_stars > 0 && det_result.n_detected > 0
            true_cents = app.cache.star_info.true_centroids;
            % binning 보정
            if strcmp(app.params.conv_method, 'binning')
                true_cents = true_cents * 0.5;
            end
            cent_result = evaluate_centroid_accuracy(det_result, true_cents);
            app.cache.centroid_rms = cent_result.rms_error;
            app.cache.centroid_matched = cent_result.n_matched;
        else
            app.cache.centroid_rms = NaN;
            app.cache.centroid_matched = 0;
        end

        app.dirty.stage3 = false;
    end

    % 렌더링
    update_images(app);
    update_metrics(app);

    app.h.lbl_status.Text = sprintf('Done (sim: %.2fs, conv: %.3fs)', ...
        app.cache.sim_time, app.cache.conv_time);

    fig.UserData = app;
end

%% ========== 이미지 렌더링 ==========
function update_images(app)
    adc_max = 2^app.params.adc_bits - 1;
    mode = app.params.display_mode;

    % 타이틀에 디스플레이 모드 표시
    mode_suffix = '';
    if ~strcmp(mode, 'RAW')
        switch mode
            case 'Digital Gain'
                mode_suffix = sprintf(' [x%d]', app.params.display_dgain);
            case 'Window'
                mode_suffix = sprintf(' [%d-%d]', ...
                    round(app.params.display_black), round(app.params.display_white));
            case 'Gamma'
                mode_suffix = sprintf(' [g=%.2f]', app.params.display_gamma);
        end
    end

    % 좌: Ideal
    ax1 = app.h.ax_ideal;
    cla(ax1);
    if ~isempty(app.cache.gray_ideal)
        raw_ideal = double(app.cache.gray_ideal);
        disp_ideal = apply_display_transform(raw_ideal, app.params, adc_max);
        imagesc(ax1, disp_ideal, [0 adc_max]);
        colormap(ax1, gray(256));
        axis(ax1, 'image');
        ax1.XTick = [];
        ax1.YTick = [];
        title(ax1, sprintf('Ideal (%dx%d, %d-bit)%s', size(raw_ideal,2), ...
            size(raw_ideal,1), app.params.adc_bits, mode_suffix), ...
            'Color', [0.5 1 0.5]);

        % 오버레이
        if app.params.show_overlay && app.cache.star_info.num_stars > 0
            hold(ax1, 'on');
            tc = app.cache.star_info.true_centroids;
            plot(ax1, tc(:,1), tc(:,2), 'go', 'MarkerSize', 10, 'LineWidth', 1.5);
            if app.params.show_labels
                mags = app.cache.star_info.magnitudes;
                [~, idx] = sort(mags);
                for i = 1:min(10, length(idx))
                    j = idx(i);
                    text(ax1, tc(j,1)+8, tc(j,2), sprintf('%.1f', mags(j)), ...
                        'Color', 'y', 'FontSize', 8);
                end
            end
            hold(ax1, 'off');
        end
    end

    % 우: Converted
    ax2 = app.h.ax_conv;
    cla(ax2);
    if ~isempty(app.cache.gray_converted)
        raw_conv = double(app.cache.gray_converted);
        disp_conv = apply_display_transform(raw_conv, app.params, adc_max);
        imagesc(ax2, disp_conv, [0 adc_max]);
        colormap(ax2, gray(256));
        axis(ax2, 'image');
        ax2.XTick = [];
        ax2.YTick = [];

        method_str = app.params.conv_method;
        title(ax2, sprintf('%s (%dx%d)%s', method_str, size(raw_conv,2), ...
            size(raw_conv,1), mode_suffix), 'Color', [1 0.8 0.4]);

        % 검출 오버레이 (최대 200개만 표시, 밝기순)
        if app.params.show_overlay && app.cache.detection.n_detected > 0
            hold(ax2, 'on');
            dc = app.cache.detection.centroids;
            di = app.cache.detection.intensities;
            max_plot = min(200, size(dc, 1));
            [~, si] = sort(di, 'descend');
            dc = dc(si(1:max_plot), :);
            plot(ax2, dc(:,1), dc(:,2), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5);
            hold(ax2, 'off');
        end
    end

    % 히스토그램 업데이트 (ON일 때만)
    if app.params.show_histogram && ~isempty(app.cache.gray_ideal)
        update_histogram(app, adc_max);
    end
end

%% ========== Display Transform ==========
function disp_img = apply_display_transform(raw_img, params, adc_max)
% APPLY_DISPLAY_TRANSFORM 디스플레이 모드에 따른 이미지 변환
% 시뮬레이션 데이터 불변 — 렌더링 전용
    switch params.display_mode
        case 'RAW'
            disp_img = raw_img;
        case 'Digital Gain'
            g = params.display_dgain;
            disp_img = min(raw_img * g, adc_max);
        case 'Window'
            black = params.display_black;
            white = max(params.display_white, black + 1);
            disp_img = (raw_img - black) / (white - black) * adc_max;
            disp_img = max(0, min(disp_img, adc_max));
        case 'Gamma'
            gamma = params.display_gamma;
            disp_img = (raw_img / adc_max) .^ gamma * adc_max;
        otherwise
            disp_img = raw_img;
    end
end

%% ========== Histogram ==========
function update_histogram(app, adc_max)
% UPDATE_HISTOGRAM RAW vs Display 히스토그램 비교 표시
    nbins = min(256, adc_max + 1);
    edges = linspace(0, adc_max, nbins + 1);
    mode = app.params.display_mode;

    % 좌: Ideal 히스토그램
    ax = app.h.ax_hist_ideal;
    cla(ax);
    raw = double(app.cache.gray_ideal);
    histogram(ax, raw(:), edges, 'FaceColor', [0.3 0.5 0.9], ...
        'FaceAlpha', 0.6, 'EdgeColor', 'none', 'DisplayName', 'RAW');
    if ~strcmp(mode, 'RAW')
        disp_img = apply_display_transform(raw, app.params, adc_max);
        hold(ax, 'on');
        histogram(ax, disp_img(:), edges, 'FaceColor', [0.9 0.4 0.3], ...
            'FaceAlpha', 0.5, 'EdgeColor', 'none', 'DisplayName', 'Display');
        legend(ax, 'TextColor', 'w', 'FontSize', 8, 'Box', 'off', 'Location', 'northeast');
        hold(ax, 'off');
    end
    ax.Color = [0.08 0.08 0.1];
    ax.XColor = [0.5 0.5 0.5];
    ax.YColor = [0.5 0.5 0.5];
    ax.YScale = 'log';
    xlim(ax, [0 adc_max]);
    title(ax, 'Ideal', 'Color', [0.5 1 0.5], 'FontSize', 9);

    % 우: Converted 히스토그램
    ax2 = app.h.ax_hist_conv;
    cla(ax2);
    if ~isempty(app.cache.gray_converted)
        raw2 = double(app.cache.gray_converted);
        histogram(ax2, raw2(:), edges, 'FaceColor', [0.3 0.5 0.9], ...
            'FaceAlpha', 0.6, 'EdgeColor', 'none', 'DisplayName', 'RAW');
        if ~strcmp(mode, 'RAW')
            disp2 = apply_display_transform(raw2, app.params, adc_max);
            hold(ax2, 'on');
            histogram(ax2, disp2(:), edges, 'FaceColor', [0.9 0.4 0.3], ...
                'FaceAlpha', 0.5, 'EdgeColor', 'none', 'DisplayName', 'Display');
            legend(ax2, 'TextColor', 'w', 'FontSize', 8, 'Box', 'off', 'Location', 'northeast');
            hold(ax2, 'off');
        end
    end
    ax2.Color = [0.08 0.08 0.1];
    ax2.XColor = [0.5 0.5 0.5];
    ax2.YColor = [0.5 0.5 0.5];
    ax2.YScale = 'log';
    xlim(ax2, [0 adc_max]);
    title(ax2, 'Converted', 'Color', [1 0.8 0.4], 'FontSize', 9);
end

%% ========== Image Area Resize ==========
function resize_image_area(app, hist_h)
% RESIZE_IMAGE_AREA 히스토그램 ON/OFF에 따라 이미지/히스토그램 위치 재조정
    fh = app.fig_h;
    L = app.layout;
    x0 = L.x0;
    img_w = L.img_w;
    gap = L.gap;
    x_right = x0 + img_w + gap;

    img_y = L.metrics_h + 10 + hist_h;
    img_h = fh - L.btn_h - L.disp_bar_h - L.metrics_h - 50 - hist_h;

    % 이미지 axes 위치/크기 변경
    app.h.ax_ideal.Position = [x0 img_y img_w img_h];
    app.h.ax_conv.Position = [x_right img_y img_w img_h];

    % 이미지 라벨 위치 변경
    app.h.lbl_ideal.Position = [x0 img_y+img_h img_w 22];
    app.h.lbl_conv.Position = [x_right img_y+img_h img_w 22];

    % 히스토그램 axes 위치
    if hist_h > 0
        hist_y = L.metrics_h + 10;
        app.h.ax_hist_ideal.Position = [x0 hist_y img_w hist_h - 5];
        app.h.ax_hist_conv.Position = [x_right hist_y img_w hist_h - 5];
    end
end

%% ========== Display Controls Visibility ==========
function update_display_controls_visibility(app)
% UPDATE_DISPLAY_CONTROLS_VISIBILITY 모드에 따라 컨트롤 표시/숨김
    mode = app.params.display_mode;

    % Digital Gain 컨트롤
    dgain_vis = strcmp(mode, 'Digital Gain');
    app.h.dd_dgain.Visible = dgain_vis;

    % Window 컨트롤
    win_vis = strcmp(mode, 'Window');
    app.h.lbl_win_black.Visible = win_vis;
    app.h.sld_win_black.Visible = win_vis;
    app.h.lbl_win_white.Visible = win_vis;
    app.h.sld_win_white.Visible = win_vis;
    app.h.lbl_win_val.Visible = win_vis;

    % Gamma 컨트롤
    gamma_vis = strcmp(mode, 'Gamma');
    app.h.lbl_gamma.Visible = gamma_vis;
    app.h.sld_gamma.Visible = gamma_vis;
    app.h.lbl_gamma_val.Visible = gamma_vis;
end

%% ========== 메트릭 업데이트 ==========
function update_metrics(app)
    p = app.params;

    % FOV
    myu = p.pixel_size * 1e-6;
    f = p.focal_length * 1e-3;
    fovx = rad2deg(2 * atan((myu * p.res_w / 2) / f));
    fovy = rad2deg(2 * atan((myu * p.res_h / 2) / f));

    app.h.lbl_m_fov.Text = sprintf('FOV: %.2f x %.2f deg', fovx, fovy);
    app.h.lbl_m_res.Text = sprintf('Resolution: %d x %d px', p.res_w, p.res_h);
    app.h.lbl_m_ifov.Text = sprintf('IFOV: %.4f deg/px', rad2deg(atan(myu/f)));
    app.h.lbl_m_pointing.Text = sprintf('Pointing: RA=%.1f, DEC=%.1f, Roll=%.1f', ...
        p.ra, p.dec, p.roll);

    % 별 정보
    if ~isempty(app.cache.star_info)
        ns = app.cache.star_info.num_stars;
        nd = app.cache.detection.n_detected;
        app.h.lbl_m_stars.Text = sprintf('Stars in FOV: %d', ns);
        if ns > 0
            rate = nd/ns*100;
        else
            rate = 0;
        end
        app.h.lbl_m_detected.Text = sprintf('Detected: %d (%.1f%%)', nd, rate);
        app.h.lbl_m_snr.Text = sprintf('Peak SNR: %.1f dB', app.cache.snr_db);
        if ~isnan(app.cache.centroid_rms)
            app.h.lbl_m_rms.Text = sprintf('Centroid RMS: %.3f px (matched %d)', ...
                app.cache.centroid_rms, app.cache.centroid_matched);
        else
            app.h.lbl_m_rms.Text = 'Centroid RMS: N/A';
        end

        % 밝은 별 목록
        if ns > 0
            [sorted_mag, idx] = sort(app.cache.star_info.magnitudes);
            lines = 'Brightest stars: ';
            for i = 1:min(8, ns)
                j = idx(i);
                tc = app.cache.star_info.true_centroids(j,:);
                lines = [lines, sprintf('  #%d: mag=%.1f (%.0f,%.0f)', ...
                    i, sorted_mag(i), tc(1), tc(2))]; %#ok<AGROW>
            end
            app.h.lbl_m_brightest.Text = lines;
        end
    end

    % 처리 정보
    app.h.lbl_m_method.Text = sprintf('Method: %s', p.conv_method);
    if isfield(app.cache, 'sim_time')
        app.h.lbl_m_simtime.Text = sprintf('Sim time: %.2f s', app.cache.sim_time);
    end
    if isfield(app.cache, 'conv_time')
        app.h.lbl_m_convtime.Text = sprintf('Conv time: %.3f s', app.cache.conv_time);
    end
    app.h.lbl_m_exposure.Text = sprintf('Exp: %.1fms | Gain: %.0fx | %s | %d-bit', ...
        p.exposure_ms, p.analog_gain, upper(p.shutter_type(1)), p.adc_bits);
end

%% ========== FOV 실시간 표시 ==========
function update_fov_display(app)
    p = app.params;
    myu = p.pixel_size * 1e-6;
    f = p.focal_length * 1e-3;
    fovx = rad2deg(2 * atan((myu * p.res_w / 2) / f));
    fovy = rad2deg(2 * atan((myu * p.res_h / 2) / f));
    fovd = sqrt(fovx^2 + fovy^2);
    ifov = rad2deg(atan(myu / f));

    app.h.lbl_fov_h.Text = sprintf('H: %.2f deg', fovx);
    app.h.lbl_fov_v.Text = sprintf('V: %.2f deg', fovy);
    app.h.lbl_fov_d.Text = sprintf('Diagonal: %.2f deg', fovd);
    app.h.lbl_ifov.Text = sprintf('IFOV: %.4f deg/px', ifov);
end

%% ========== 노이즈 버짓 실시간 ==========
function update_noise_budget(app)
    p = app.params;
    exp_s = p.exposure_ms * 1e-3;
    total_gain = p.analog_gain * p.digital_gain;

    % ADC 최대값
    adc_max = 2^p.adc_bits - 1;

    % 환경별 광자 플럭스
    if strcmp(p.environment, 'space')
        ref_flux = 96 / 0.56;  % 대기 보정 역산
    else
        ref_flux = 96;
    end
    ref_mag = 6.0;

    % Arrhenius 다크 전류
    dark_rate = p.dark_current * 2^((p.sensor_temp - 25) / 6.5);

    % 신호 추정 (Pogson)
    for mag = [1 3 6]
        pf = ref_flux * 10^(-0.4 * (mag - ref_mag));
        electrons = pf * exp_s * p.qe;
        adu = min(adc_max, electrons * total_gain);
        switch mag
            case 1, app.h.lbl_sig1.Text = sprintf('1st mag: %.0f ADU (%.0f e-)', adu, electrons);
            case 3, app.h.lbl_sig3.Text = sprintf('3rd mag: %.0f ADU (%.0f e-)', adu, electrons);
            case 6, app.h.lbl_sig6.Text = sprintf('6th mag: %.0f ADU (%.0f e-)', adu, electrons);
        end
    end
    app.h.lbl_sigsat.Text = sprintf('Saturation: %d ADU (%d-bit)', adc_max, p.adc_bits);

    % 노이즈 성분
    sig6_e = ref_flux * exp_s * p.qe;
    shot_adu = sqrt(sig6_e) * total_gain;
    read_adu = p.read_noise * total_gain;
    dark_e = dark_rate * exp_s;
    dark_adu = sqrt(dark_e) * total_gain;
    total_adu = sqrt(shot_adu^2 + read_adu^2 + dark_adu^2);

    app.h.lbl_nshot.Text = sprintf('Shot noise (6mag): %.1f ADU', shot_adu);
    app.h.lbl_nread.Text = sprintf('Read noise (ADU):  %.1f ADU', read_adu);
    app.h.lbl_ndark.Text = sprintf('Dark @%dC: %.3f ADU', p.sensor_temp, dark_adu);
    app.h.lbl_ntotal.Text = sprintf('Total noise RMS:   %.1f ADU', total_adu);

    % Exposure 라벨
    app.h.lbl_exp.Text = sprintf('%.1f ms', p.exposure_ms);
    app.h.lbl_gain.Text = sprintf('%.1fx', p.analog_gain);

    % Motion info 업데이트
    update_motion_info(app);
end

%% ========== Motion Info 실시간 ==========
function update_motion_info(app)
    p = app.params;
    if ~isfield(app.h, 'lbl_mot_shutter'), return; end

    % Shutter type
    if strcmp(p.shutter_type, 'rolling')
        app.h.lbl_mot_shutter.Text = sprintf('Shutter: Rolling (readout %.0fms)', ...
            p.readout_time * 1000);
    else
        app.h.lbl_mot_shutter.Text = 'Shutter: Global';
    end

    % Motion blur
    myu = p.pixel_size * 1e-6;
    f = p.focal_length * 1e-3;
    exp_s = p.exposure_ms * 1e-3;
    omega_rad = deg2rad([p.omega_x, p.omega_y, p.omega_z]);
    blur_dx = omega_rad(2) * (f / myu) * exp_s;
    blur_dy = -omega_rad(1) * (f / myu) * exp_s;
    blur_len = sqrt(blur_dx^2 + blur_dy^2);
    app.h.lbl_mot_blur.Text = sprintf('Motion blur: %.1f px', blur_len);

    % RS max shift
    if strcmp(p.shutter_type, 'rolling') && p.readout_time > 0
        rs_dx = omega_rad(2) * (f / myu) * p.readout_time;
        rs_dy = -omega_rad(1) * (f / myu) * p.readout_time;
        rs_max = sqrt(rs_dx^2 + rs_dy^2);
        app.h.lbl_mot_rs.Text = sprintf('RS max shift: %.1f px', rs_max);
    else
        app.h.lbl_mot_rs.Text = 'RS max shift: N/A (Global)';
    end

    % ADC / PSF
    adc_max = 2^p.adc_bits - 1;
    app.h.lbl_mot_adc.Text = sprintf('ADC: %d-bit (0-%d)', p.adc_bits, adc_max);
    app.h.lbl_mot_psf.Text = sprintf('PSF sigma: %.2f px', p.psf_sigma);

    % Sensor info
    if isfield(app.h, 'lbl_sensor_info')
        app.h.lbl_sensor_info.Text = sprintf('%s | %s | %d-bit | FW=%de-', ...
            p.sensor_preset, upper(p.shutter_type(1:2)), p.adc_bits, p.full_well);
    end
end

%% ================================================================
%% =================== CALLBACKS ==================================
%% ================================================================

% --- Sensor callbacks ---
function cb_sensor(src, ~, param, fig)
    app = fig.UserData;
    val = src.Value;
    switch param
        case 'pixel_size'
            app.params.pixel_size = val;
            app.h.edit_pix.Value = val;
        case 'focal_length'
            app.params.focal_length = val;
            app.h.edit_fl.Value = val;
            app.params.lens_preset = 'Custom';
            app.h.dd_lens.Value = 'Custom';
        case 'qe'
            app.params.qe = val;
            app.h.edit_qe.Value = val;
        case 'mag_limit'
            app.params.mag_limit = val;
            app.h.edit_mag.Value = val;
    end
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_fov_display(app);
    update_noise_budget(app);
end

function cb_sensor_edit(src, ~, param, fig)
    app = fig.UserData;
    val = src.Value;
    switch param
        case 'pixel_size'
            app.params.pixel_size = val;
            app.h.slider_pix.Value = val;
        case 'focal_length'
            app.params.focal_length = val;
            app.h.slider_fl.Value = val;
        case 'qe'
            app.params.qe = val;
            app.h.slider_qe.Value = val;
        case 'mag_limit'
            app.params.mag_limit = val;
            app.h.slider_mag.Value = val;
    end
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_fov_display(app);
    update_noise_budget(app);
end

function cb_res_preset(~, ~, w, h, fig)
    app = fig.UserData;
    app.params.res_w = w;
    app.params.res_h = h;
    app.h.edit_resw.Value = w;
    app.h.edit_resh.Value = h;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_fov_display(app);
end

function cb_res_changed(~, ~, fig)
    app = fig.UserData;
    app.params.res_w = round(app.h.edit_resw.Value);
    app.params.res_h = round(app.h.edit_resh.Value);
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_fov_display(app);
end

% --- Exposure callbacks ---
function cb_exp_slider(src, ~, fig)
    app = fig.UserData;
    val_ms = 2^src.Value;
    val_ms = max(1, min(500, val_ms));
    app.params.exposure_ms = val_ms;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_exp_preset(~, ~, val_ms, fig)
    app = fig.UserData;
    app.params.exposure_ms = val_ms;
    app.h.slider_exp.Value = log2(val_ms);
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_gain_preset(~, ~, gain, fig)
    app = fig.UserData;
    app.params.analog_gain = gain;
    app.h.slider_gain.Value = gain;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_gain_adjust(~, ~, delta, fig)
    app = fig.UserData;
    new_gain = max(1, min(64, app.params.analog_gain + delta));
    app.params.analog_gain = new_gain;
    app.h.slider_gain.Value = new_gain;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_gain_slider(src, ~, fig)
    app = fig.UserData;
    app.params.analog_gain = round(src.Value);
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_dgain(src, ~, fig)
    app = fig.UserData;
    app.params.digital_gain = src.Value;
    app.h.edit_dgain.Value = src.Value;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_dgain_edit(src, ~, fig)
    app = fig.UserData;
    app.params.digital_gain = src.Value;
    app.h.slider_dgain.Value = src.Value;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

% --- Noise callbacks ---
function cb_noise_toggle(src, ~, fig)
    app = fig.UserData;
    app.params.noise_enabled = src.Value;
    app.dirty.stage1 = true;
    fig.UserData = app;
end

function cb_noise_param(src, ~, param, fig)
    app = fig.UserData;
    switch param
        case 'dark_current'
            app.params.dark_current = src.Value;
            app.h.edit_dark.Value = src.Value;
        case 'read_noise'
            app.params.read_noise = src.Value;
            app.h.edit_read.Value = src.Value;
    end
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_noise_param_edit(src, ~, param, fig)
    app = fig.UserData;
    switch param
        case 'dark_current'
            app.params.dark_current = src.Value;
            app.h.slider_dark.Value = src.Value;
        case 'read_noise'
            app.params.read_noise = src.Value;
            app.h.slider_read.Value = src.Value;
    end
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_dark_preset(~, ~, val, fig)
    app = fig.UserData;
    app.params.dark_current = val;
    app.h.slider_dark.Value = val;
    app.h.edit_dark.Value = val;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_scene_preset(~, ~, preset, fig)
    app = fig.UserData;
    switch preset
        case 'space'
            app.params.exposure_ms = 22;
            app.params.analog_gain = 16;
            app.params.digital_gain = 1.0;
            app.params.dark_current = 0.01;
            app.params.read_noise = 3;
            app.params.noise_enabled = true;
        case 'ground'
            app.params.exposure_ms = 100;
            app.params.analog_gain = 4;
            app.params.digital_gain = 1.0;
            app.params.dark_current = 0.1;
            app.params.read_noise = 3;
            app.params.noise_enabled = true;
        case 'low_noise'
            app.params.exposure_ms = 50;
            app.params.analog_gain = 8;
            app.params.digital_gain = 1.0;
            app.params.dark_current = 0;
            app.params.read_noise = 0;
            app.params.noise_enabled = false;
        case 'high_noise'
            app.params.exposure_ms = 22;
            app.params.analog_gain = 64;
            app.params.digital_gain = 2.0;
            app.params.dark_current = 1.0;
            app.params.read_noise = 15;
            app.params.noise_enabled = true;
    end
    % UI 동기화
    app.h.slider_exp.Value = log2(max(1, app.params.exposure_ms));
    app.h.slider_gain.Value = app.params.analog_gain;
    app.h.slider_dgain.Value = app.params.digital_gain;
    app.h.edit_dgain.Value = app.params.digital_gain;
    app.h.slider_dark.Value = app.params.dark_current;
    app.h.edit_dark.Value = app.params.dark_current;
    app.h.slider_read.Value = app.params.read_noise;
    app.h.edit_read.Value = app.params.read_noise;
    app.h.chk_noise.Value = app.params.noise_enabled;

    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

% --- Observation callbacks ---
function cb_obs(src, ~, param, fig)
    app = fig.UserData;
    switch param
        case 'ra'
            app.params.ra = src.Value;
            app.h.edit_ra.Value = src.Value;
        case 'dec'
            app.params.dec = src.Value;
            app.h.edit_dec.Value = src.Value;
        case 'roll'
            app.params.roll = src.Value;
            app.h.edit_roll.Value = src.Value;
    end
    app.dirty.stage1 = true;
    fig.UserData = app;
end

function cb_obs_edit(src, ~, param, fig)
    app = fig.UserData;
    switch param
        case 'ra'
            app.params.ra = src.Value;
            app.h.slider_ra.Value = src.Value;
        case 'dec'
            app.params.dec = src.Value;
            app.h.slider_dec.Value = src.Value;
        case 'roll'
            app.params.roll = src.Value;
            app.h.slider_roll.Value = src.Value;
    end
    app.dirty.stage1 = true;
    fig.UserData = app;
end

function cb_obs_preset(~, ~, ra, dec, fig)
    app = fig.UserData;
    if ra < 0  % Random
        ra = rand() * 360;
        dec = rand() * 180 - 90;
    end
    app.params.ra = ra;
    app.params.dec = dec;
    app.h.slider_ra.Value = ra;
    app.h.edit_ra.Value = ra;
    app.h.slider_dec.Value = dec;
    app.h.edit_dec.Value = dec;
    app.dirty.stage1 = true;
    fig.UserData = app;
end

% --- Processing callbacks ---
function cb_method_changed(src, ~, fig)
    app = fig.UserData;
    app.params.conv_method = src.Value;
    app.h.panel_weights.Visible = strcmp(src.Value, 'optimal');
    app.dirty.stage2 = true;
    fig.UserData = app;
end

function cb_weight_changed(~, ~, fig)
    app = fig.UserData;
    w_R = app.h.slider_wR.Value;
    w_G = app.h.slider_wG.Value;
    w_B = app.h.slider_wB.Value;
    total = w_R + w_G + w_B;

    app.h.lbl_wR.Text = sprintf('%.4f', w_R);
    app.h.lbl_wG.Text = sprintf('%.4f', w_G);
    app.h.lbl_wB.Text = sprintf('%.4f', w_B);
    app.h.lbl_wsum.Text = sprintf('Sum: %.3f', total);

    app.params.opt_weights = [w_R, w_G, w_B];
    app.dirty.stage2 = true;
    fig.UserData = app;
end

function cb_normalize_weights(~, ~, fig)
    app = fig.UserData;
    w = app.params.opt_weights;
    total = sum(w);
    if total > 0
        w = w / total;
    end
    app.params.opt_weights = w;
    app.h.slider_wR.Value = w(1);
    app.h.slider_wG.Value = w(2);
    app.h.slider_wB.Value = w(3);
    app.h.lbl_wR.Text = sprintf('%.4f', w(1));
    app.h.lbl_wG.Text = sprintf('%.4f', w(2));
    app.h.lbl_wB.Text = sprintf('%.4f', w(3));
    app.h.lbl_wsum.Text = sprintf('Sum: %.3f', sum(w));
    app.dirty.stage2 = true;
    fig.UserData = app;
end

function cb_reset_weights(~, ~, fig)
    app = fig.UserData;
    w = [0.4544, 0.3345, 0.2111];
    app.params.opt_weights = w;
    app.h.slider_wR.Value = w(1);
    app.h.slider_wG.Value = w(2);
    app.h.slider_wB.Value = w(3);
    app.h.lbl_wR.Text = sprintf('%.4f', w(1));
    app.h.lbl_wG.Text = sprintf('%.4f', w(2));
    app.h.lbl_wB.Text = sprintf('%.4f', w(3));
    app.h.lbl_wsum.Text = sprintf('Sum: %.3f', sum(w));
    app.dirty.stage2 = true;
    fig.UserData = app;
end

function cb_detect_param(src, ~, param, fig)
    app = fig.UserData;
    val = round(src.Value);
    switch param
        case 'threshold'
            app.params.threshold = val;
            app.h.edit_thresh.Value = val;
        case 'min_area'
            app.params.min_area = val;
            app.h.edit_minarea.Value = val;
    end
    app.dirty.stage3 = true;
    fig.UserData = app;
end

function cb_detect_param_edit(src, ~, param, fig)
    app = fig.UserData;
    val = round(src.Value);
    switch param
        case 'threshold'
            app.params.threshold = val;
            app.h.slider_thresh.Value = val;
        case 'min_area'
            app.params.min_area = val;
            app.h.slider_minarea.Value = val;
    end
    app.dirty.stage3 = true;
    fig.UserData = app;
end

function cb_display_opt(~, ~, fig)
    app = fig.UserData;
    app.params.show_overlay = app.h.chk_overlay.Value;
    app.params.show_labels = app.h.chk_labels.Value;
    fig.UserData = app;
    % 이미지만 다시 그리기 (시뮬레이션 불필요)
    if ~isempty(app.cache.gray_ideal)
        update_images(app);
    end
end

% --- Action buttons ---
function cb_simulate(~, ~, fig)
    run_simulation(fig, true);
end

function cb_quick_update(~, ~, fig)
    app = fig.UserData;
    if isempty(app.cache.bayer_img)
        app.h.lbl_status.Text = 'No cached data. Run [Simulate] first!';
        fig.UserData = app;
        return;
    end
    app.dirty.stage1 = false;
    app.dirty.stage2 = true;
    fig.UserData = app;
    run_simulation(fig, false);
end

function cb_export(~, ~, fig)
    app = fig.UserData;
    if isempty(app.cache.gray_ideal)
        app.h.lbl_status.Text = 'No images to export. Run [Simulate] first!';
        return;
    end

    % 타임스탬프 (파일명용)
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

    % 디렉토리 생성
    script_dir = fileparts(mfilename('fullpath'));
    img_dir = fullfile(script_dir, 'output', 'img');
    coord_dir = fullfile(script_dir, 'output', 'coordinate');
    if ~exist(img_dir, 'dir'), mkdir(img_dir); end
    if ~exist(coord_dir, 'dir'), mkdir(coord_dir); end

    % 1. 이미지 저장 (16-bit PNG for 10/12-bit sensors)
    adc_max = 2^app.params.adc_bits - 1;
    ideal_norm = uint16(double(app.cache.gray_ideal) / adc_max * 65535);
    imwrite(ideal_norm, fullfile(img_dir, sprintf('%s_ideal.png', timestamp)));
    conv_norm = uint16(double(app.cache.gray_converted) / adc_max * 65535);
    imwrite(conv_norm, fullfile(img_dir, sprintf('%s_converted_%s.png', timestamp, app.params.conv_method)));

    % 2. True 좌표 저장 (시뮬레이션 참값)
    if app.cache.star_info.num_stars > 0
        tc = app.cache.star_info.true_centroids;
        mag = app.cache.star_info.magnitudes;
        fid = fopen(fullfile(coord_dir, sprintf('%s_true_coords.csv', timestamp)), 'w');
        fprintf(fid, 'x,y,magnitude\n');
        for i = 1:size(tc, 1)
            fprintf(fid, '%.4f,%.4f,%.2f\n', tc(i,1), tc(i,2), mag(i));
        end
        fclose(fid);
    end

    % 3. Detected 좌표 저장 (검출 결과)
    if app.cache.detection.n_detected > 0
        dc = app.cache.detection.centroids;
        di = app.cache.detection.intensities;
        fid = fopen(fullfile(coord_dir, sprintf('%s_detected_coords.csv', timestamp)), 'w');
        fprintf(fid, 'x,y,intensity\n');
        for i = 1:size(dc, 1)
            fprintf(fid, '%.4f,%.4f,%.1f\n', dc(i,1), dc(i,2), di(i));
        end
        fclose(fid);
    end

    % 상태 메시지
    n_true = app.cache.star_info.num_stars;
    n_det = app.cache.detection.n_detected;
    app.h.lbl_status.Text = sprintf('Exported: 2 imgs + coords (true:%d, det:%d) [%s]', ...
        n_true, n_det, timestamp);
    fig.UserData = app;
end

%% ========== Display Pipeline Callbacks ==========
function cb_display_mode(src, ~, fig)
    app = fig.UserData;
    app.params.display_mode = src.Value;
    update_display_controls_visibility(app);
    if ~isempty(app.cache.gray_ideal)
        update_images(app);
    end
    fig.UserData = app;
end

function cb_display_dgain(src, ~, fig)
    app = fig.UserData;
    val_str = src.Value;  % 'x1', 'x2', 'x4', 'x8', 'x16'
    app.params.display_dgain = str2double(val_str(2:end));
    if ~isempty(app.cache.gray_ideal)
        update_images(app);
    end
    fig.UserData = app;
end

function cb_display_window(~, ~, fig)
    app = fig.UserData;
    app.params.display_black = app.h.sld_win_black.Value;
    app.params.display_white = app.h.sld_win_white.Value;
    app.h.lbl_win_val.Text = sprintf('[%d - %d]', ...
        round(app.params.display_black), round(app.params.display_white));
    if ~isempty(app.cache.gray_ideal)
        update_images(app);
    end
    fig.UserData = app;
end

function cb_display_gamma(~, ~, fig)
    app = fig.UserData;
    app.params.display_gamma = app.h.sld_gamma.Value;
    app.h.lbl_gamma_val.Text = sprintf('%.2f', app.params.display_gamma);
    if ~isempty(app.cache.gray_ideal)
        update_images(app);
    end
    fig.UserData = app;
end

function cb_histogram_toggle(~, ~, fig)
    app = fig.UserData;
    app.params.show_histogram = ~app.params.show_histogram;

    if app.params.show_histogram
        hist_h = 120;
        app.h.ax_hist_ideal.Visible = 'on';
        app.h.ax_hist_conv.Visible = 'on';
        app.h.btn_histogram.BackgroundColor = [0.3 0.5 0.3];  % 활성 색상
        app.h.btn_histogram.Text = 'Histogram ON';
    else
        hist_h = 0;
        app.h.ax_hist_ideal.Visible = 'off';
        app.h.ax_hist_conv.Visible = 'off';
        app.h.btn_histogram.BackgroundColor = [0.25 0.25 0.3];
        app.h.btn_histogram.Text = 'Histogram';
    end

    resize_image_area(app, hist_h);

    if ~isempty(app.cache.gray_ideal)
        update_images(app);
    end
    fig.UserData = app;
end

%% ========== Sensor/Lens Preset Callbacks ==========
function cb_sensor_preset(src, ~, fig)
    app = fig.UserData;
    sensor_name = src.Value;

    % 구분선 선택 무시
    if startsWith(sensor_name, '---')
        src.Value = app.params.sensor_preset;  % 이전 값 복원
        return;
    end

    app.params.sensor_preset = sensor_name;

    if strcmp(sensor_name, 'Custom')
        % Custom: 슬라이더 값 유지, 변경 없음
        fig.UserData = app;
        return;
    end

    % OV4689는 기존 기본값 사용
    if strcmp(sensor_name, 'OV4689')
        app.params.pixel_size = 2.0;
        app.params.res_w = 1280;
        app.params.res_h = 720;
        app.params.qe = 0.5;
        app.params.read_noise = 3.0;
        app.params.dark_current = 0.1;
        app.params.shutter_type = 'global';
        app.params.readout_time = 0;
        app.params.adc_bits = 8;
        app.params.full_well = 10000;
        app.params.psf_sigma = 1.2;
        app.params.sens_R = 0.916;
        app.params.sens_G = 1.0;
        app.params.sens_B = 0.854;
        app.params.analog_gain = 16;
    else
        % get_sensor_preset 로드 (렌즈는 현재 선택된 것 사용)
        lens = app.params.lens_preset;
        if strcmp(lens, 'Custom')
            lens = '16mm';  % 기본 렌즈
        end
        try
            sp = get_sensor_preset(sensor_name, lens);
            app.params.pixel_size = sp.myu * 1e6;
            app.params.res_w = sp.l;
            app.params.res_h = sp.w;
            app.params.qe = sp.quantum_efficiency;
            app.params.read_noise = sp.read_noise;
            app.params.dark_current = sp.dark_current_ref;
            app.params.shutter_type = sp.shutter_type;
            app.params.readout_time = sp.readout_time;
            app.params.adc_bits = sp.adc_bits;
            app.params.full_well = sp.full_well;
            app.params.psf_sigma = sp.psf_sigma;
            app.params.sens_R = sp.sensitivity_R;
            app.params.sens_G = sp.sensitivity_G;
            app.params.sens_B = sp.sensitivity_B;
            app.params.analog_gain = sp.analog_gain;
            app.params.focal_length = sp.f * 1e3;
            app.params.lens_preset = lens;
            app.h.dd_lens.Value = lens;
        catch ME
            app.h.lbl_status.Text = sprintf('Preset error: %s', ME.message);
            fig.UserData = app;
            return;
        end
    end

    % Window 모드 슬라이더 범위를 ADC 비트에 연동
    adc_max = 2^app.params.adc_bits - 1;
    app.params.display_white = adc_max;
    app.params.display_black = 0;
    app.h.sld_win_black.Limits = [0 adc_max];
    app.h.sld_win_black.Value = 0;
    app.h.sld_win_white.Limits = [0 adc_max];
    app.h.sld_win_white.Value = adc_max;
    app.h.lbl_win_val.Text = sprintf('[0 - %d]', adc_max);

    % UI 동기화
    sync_sensor_ui(app);
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_fov_display(app);
    update_noise_budget(app);
end

function cb_lens_preset(src, ~, fig)
    app = fig.UserData;
    lens_name = src.Value;
    app.params.lens_preset = lens_name;

    if strcmp(lens_name, 'Custom')
        fig.UserData = app;
        return;
    end

    sensor = app.params.sensor_preset;
    if strcmp(sensor, 'Custom') || strcmp(sensor, 'OV4689')
        % OV4689/Custom: 렌즈 focal length만 변경
        try
            sp = get_sensor_preset('IMX296', lens_name);  % 렌즈 정보만 가져옴
            app.params.focal_length = sp.f * 1e3;
            % PSF도 업데이트 (현재 센서 픽셀로 재계산)
            myu = app.params.pixel_size * 1e-6;
            airy_r = 1.22 * 550e-9 * sp.f_number;
            app.params.psf_sigma = max(0.8, airy_r / myu);
        catch ME
            app.h.lbl_status.Text = sprintf('Lens error: %s', ME.message);
            fig.UserData = app;
            return;
        end
    else
        % IMX 센서: 전체 프리셋 로드
        try
            sp = get_sensor_preset(sensor, lens_name);
            app.params.focal_length = sp.f * 1e3;
            app.params.psf_sigma = sp.psf_sigma;
        catch ME
            app.h.lbl_status.Text = sprintf('Preset error: %s', ME.message);
            fig.UserData = app;
            return;
        end
    end

    % UI 동기화
    app.h.slider_fl.Value = max(5, min(50, app.params.focal_length));
    app.h.edit_fl.Value = app.params.focal_length;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_fov_display(app);
    update_noise_budget(app);
end

function sync_sensor_ui(app)
    % 모든 센서 관련 UI를 현재 params에 동기화
    p = app.params;
    app.h.slider_pix.Value = max(1, min(10, p.pixel_size));
    app.h.edit_pix.Value = p.pixel_size;
    app.h.slider_fl.Value = max(5, min(50, p.focal_length));
    app.h.edit_fl.Value = p.focal_length;
    app.h.edit_resw.Value = p.res_w;
    app.h.edit_resh.Value = p.res_h;
    app.h.slider_qe.Value = max(0.1, min(0.9, p.qe));
    app.h.edit_qe.Value = p.qe;
    app.h.slider_read.Value = max(0, min(20, p.read_noise));
    app.h.edit_read.Value = p.read_noise;
    app.h.slider_dark.Value = max(0, min(1, p.dark_current));
    app.h.edit_dark.Value = p.dark_current;
    app.h.slider_gain.Value = max(1, min(64, p.analog_gain));
end

%% ========== Motion Callbacks ==========
function cb_motion(src, ~, param, fig)
    app = fig.UserData;
    val = src.Value;
    app.params.(param) = val;
    switch param
        case 'omega_x', app.h.edit_ox.Value = val;
        case 'omega_y', app.h.edit_oy.Value = val;
        case 'omega_z', app.h.edit_oz.Value = val;
    end
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_motion_info(app);
end

function cb_motion_edit(src, ~, param, fig)
    app = fig.UserData;
    val = src.Value;
    app.params.(param) = val;
    switch param
        case 'omega_x', app.h.slider_ox.Value = val;
        case 'omega_y', app.h.slider_oy.Value = val;
        case 'omega_z', app.h.slider_oz.Value = val;
    end
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_motion_info(app);
end

function cb_motion_preset(~, ~, omega, fig)
    app = fig.UserData;
    app.params.omega_x = omega(1);
    app.params.omega_y = omega(2);
    app.params.omega_z = omega(3);
    app.h.slider_ox.Value = omega(1);
    app.h.edit_ox.Value = omega(1);
    app.h.slider_oy.Value = omega(2);
    app.h.edit_oy.Value = omega(2);
    app.h.slider_oz.Value = omega(3);
    app.h.edit_oz.Value = omega(3);
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_motion_info(app);
end

function cb_preview_scale(~, ~, scale, fig)
    app = fig.UserData;
    app.params.preview_scale = scale;
    if scale > 1
        app.h.lbl_preview.Text = sprintf('(%dx%d)', ...
            round(app.params.res_w/scale), round(app.params.res_h/scale));
        % 선택 버튼 강조
        app.h.btn_scale1.BackgroundColor = [0.96 0.96 0.96];
        app.h.btn_scale2.BackgroundColor = [0.96 0.96 0.96];
        app.h.btn_scale4.BackgroundColor = [0.96 0.96 0.96];
        if scale == 2
            app.h.btn_scale2.BackgroundColor = [0.3 0.5 0.3];
        else
            app.h.btn_scale4.BackgroundColor = [0.3 0.5 0.3];
        end
    else
        app.h.lbl_preview.Text = '';
        app.h.btn_scale1.BackgroundColor = [0.3 0.5 0.3];
        app.h.btn_scale2.BackgroundColor = [0.96 0.96 0.96];
        app.h.btn_scale4.BackgroundColor = [0.96 0.96 0.96];
    end
    app.dirty.stage1 = true;
    fig.UserData = app;
end

function cb_env_changed(src, ~, fig)
    app = fig.UserData;
    app.params.environment = src.Value;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end

function cb_temp_changed(src, ~, fig)
    app = fig.UserData;
    app.params.sensor_temp = src.Value;
    app.dirty.stage1 = true;
    fig.UserData = app;
    update_noise_budget(app);
end
