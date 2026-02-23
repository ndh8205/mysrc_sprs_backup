function sensor_params = get_sensor_preset(sensor_name, lens_name, varargin)
% GET_SENSOR_PRESET 센서+렌즈 조합의 프리셋 파라미터 반환
%
% 입력:
%   sensor_name - 센서 이름 (아래 목록 참조)
%   lens_name   - 렌즈 이름: '6mm', '8mm', '12mm', '16mm', '25mm', '8-50mm'
%   varargin    - Name-Value 쌍으로 개별 파라미터 오버라이드
%
% 지원 센서 (18종):
%   Sony Starvis 2 (BSI): IMX585, IMX678, IMX662
%   Sony Starvis   (BSI): IMX485, IMX462
%   Sony Pregius   (GS):  IMX174, IMX249, IMX264, IMX250, IMX252
%   Sony (기타):          IMX296, IMX477, IMX219
%   ams-OSRAM (GS):       CMV4000, CMV2000
%   OmniVision:           OV9281
%   onsemi:               MT9P031, AR0134
%
% 사용 예시:
%   sp = get_sensor_preset('IMX585', '25mm');
%   sp = get_sensor_preset('CMV4000', '16mm', 'environment', 'ground');
%
% 센서 사양 출처:
%   Sony 제품 사양서, ZWO/QHYCCD 천문카메라 데이터시트,
%   ams-OSRAM CMV 데이터시트, FRAMOS 센서 비교 자료,
%   GAiMSat-1 COTS 센서 비교 문서 (cots_sensor_comparison.html)

%% 센서 프리셋
switch upper(sensor_name)

    % ============================================================
    %  Sony Starvis 2 (BSI, 고감도)
    % ============================================================
    case 'IMX585'
        % === Sony IMX585 - Starvis 2, BSI ===
        % 기준 센서. 최고 SNR (47ke- FW, 90% QE)
        % 궤도 실적 없음. 자체 TID/SEE 시험 필요.
        sp = struct();
        sp.sensor_name = 'IMX585';
        sp.l = 3856;              % 가로 픽셀
        sp.w = 2180;              % 세로 픽셀
        sp.myu = 2.9e-6;          % 픽셀 크기 [m] (2.9 um)
        sp.quantum_efficiency = 0.90;  % QE peak ~90%
        sp.read_noise = 1.0;      % 읽기 노이즈 [e- RMS]
        sp.full_well = 47000;     % Full well capacity [e-]
        sp.dark_current_ref = 0.05; % 다크전류 @25C [e-/px/s]
        sp.adc_bits = 12;
        sp.shutter_type = 'rolling';
        sp.readout_time = 0.037;  % ~37ms (27fps @12-bit)
        sp.sensitivity_R = 0.92;  % Starvis 2: NIR 감도 증가 → R 높음
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.78;
        sp.exposure_time = 0.050;
        sp.analog_gain = 4.0;     % 47ke FW + 12bit, mag0→~59% fill
        sp.digital_gain = 1.0;

    case 'IMX678'
        % === Sony IMX678 - Starvis 2, BSI ===
        % 4K 해상도, 소형 1/1.8" → 렌즈 비용 절감
        % HCG 모드: read noise 0.6e-
        sp = struct();
        sp.sensor_name = 'IMX678';
        sp.l = 3840;
        sp.w = 2160;
        sp.myu = 2.0e-6;          % 2.0 um
        sp.quantum_efficiency = 0.83;
        sp.read_noise = 1.0;      % ~1.0e- (HCG 모드 @gain 80)
        sp.full_well = 11270;     % ~11.3 ke-
        sp.dark_current_ref = 0.08;
        sp.adc_bits = 12;
        sp.shutter_type = 'rolling';
        sp.readout_time = 0.021;  % ~21ms (47.5fps @12-bit)
        sp.sensitivity_R = 0.88;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.72;  % 소형 픽셀: Blue 감도 낮음
        sp.exposure_time = 0.050;
        sp.analog_gain = 5.0;     % 11.3ke FW + 12bit, 소형 2um → mag0→~50% fill
        sp.digital_gain = 1.0;

    case 'IMX662'
        % === Sony IMX662 - Starvis 2, BSI ===
        % 2MP Full HD. QE 91%, FW 38ke- (픽셀 대비 매우 큼)
        % 초저전력, 고속 (102fps)
        sp = struct();
        sp.sensor_name = 'IMX662';
        sp.l = 1920;
        sp.w = 1080;
        sp.myu = 2.9e-6;          % 2.9 um
        sp.quantum_efficiency = 0.91;
        sp.read_noise = 1.2;      % ~1.2e- (HCG)
        sp.full_well = 38000;     % ~38 ke-
        sp.dark_current_ref = 0.08;
        sp.adc_bits = 12;
        sp.shutter_type = 'rolling';
        sp.readout_time = 0.010;  % ~10ms (102fps)
        sp.sensitivity_R = 0.90;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.75;
        sp.exposure_time = 0.050;
        sp.analog_gain = 6.0;     % 38ke FW + 12bit
        sp.digital_gain = 1.0;

    % ============================================================
    %  Sony Starvis (BSI, 1세대)
    % ============================================================
    case 'IMX485'
        % === Sony IMX485 - Starvis, BSI ===
        % IMX585의 전세대. 동일 1/1.2" 포맷, 2.9um 픽셀.
        % amp glow 있음 (장노출 시 주의)
        sp = struct();
        sp.sensor_name = 'IMX485';
        sp.l = 3864;
        sp.w = 2180;
        sp.myu = 2.9e-6;
        sp.quantum_efficiency = 0.85;
        sp.read_noise = 1.5;      % ~1.5e- (sHCG)
        sp.full_well = 13000;     % ~13 ke-
        sp.dark_current_ref = 0.15;
        sp.adc_bits = 12;
        sp.shutter_type = 'rolling';
        sp.readout_time = 0.037;  % ~37ms (27fps)
        sp.sensitivity_R = 0.85;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.72;
        sp.exposure_time = 0.050;
        sp.analog_gain = 6.0;     % 13ke FW + 12bit, mag0→~45% fill(NF)
        sp.digital_gain = 1.0;

    case 'IMX462'
        % === Sony IMX462 - Starvis, BSI ===
        % IMX662의 전세대. 2MP Full HD. NIR 감도 강점.
        % 최저가 ($5~12)
        sp = struct();
        sp.sensor_name = 'IMX462';
        sp.l = 1920;
        sp.w = 1080;
        sp.myu = 2.9e-6;
        sp.quantum_efficiency = 0.80;
        sp.read_noise = 2.5;
        sp.full_well = 15000;     % ~15 ke-
        sp.dark_current_ref = 0.15;
        sp.adc_bits = 12;
        sp.shutter_type = 'rolling';
        sp.readout_time = 0.011;  % ~11ms (90fps)
        sp.sensitivity_R = 0.85;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.70;
        sp.exposure_time = 0.050;
        sp.analog_gain = 8.0;     % Starvis 1: QE 낮음 보상
        sp.digital_gain = 1.0;

    % ============================================================
    %  Sony Pregius (Global Shutter)
    % ============================================================
    case 'IMX174'
        % === Sony IMX174 - Pregius 1세대, Global Shutter ===
        % 대형 픽셀 5.86um → 높은 SNR. DR 73dB, 166fps.
        % LVDS 인터페이스 (MIPI 아님)
        sp = struct();
        sp.sensor_name = 'IMX174';
        sp.l = 1920;
        sp.w = 1200;
        sp.myu = 5.86e-6;         % 5.86 um (대형)
        sp.quantum_efficiency = 0.77;
        sp.read_noise = 5.0;      % Pregius 1세대 GS
        sp.full_well = 32000;     % >30 ke-
        sp.dark_current_ref = 0.1;
        sp.adc_bits = 12;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 0.82;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.68;  % FSI 기반 Pregius: Blue 낮음
        sp.exposure_time = 0.050;
        sp.analog_gain = 4.0;     % 32ke FW + 5.86um 대형 → mag0→~57% fill
        sp.digital_gain = 1.0;

    case 'IMX249'
        % === Sony IMX249 - Pregius 1세대 저가형, Global Shutter ===
        % IMX174 동일 화질, 41fps (속도 ↓), 가격 ↓
        sp = struct();
        sp.sensor_name = 'IMX249';
        sp.l = 1920;
        sp.w = 1200;
        sp.myu = 5.86e-6;
        sp.quantum_efficiency = 0.77;
        sp.read_noise = 5.0;
        sp.full_well = 32000;
        sp.dark_current_ref = 0.1;
        sp.adc_bits = 12;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 0.82;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.68;
        sp.exposure_time = 0.050;
        sp.analog_gain = 4.0;     % IMX174 동일 사양
        sp.digital_gain = 1.0;

    case 'IMX264'
        % === Sony IMX264 - Pregius 2세대, Global Shutter ===
        % CCD 대체용 설계. 3.45um, 2/3" 포맷.
        sp = struct();
        sp.sensor_name = 'IMX264';
        sp.l = 2448;
        sp.w = 2048;
        sp.myu = 3.45e-6;
        sp.quantum_efficiency = 0.72;
        sp.read_noise = 3.5;
        sp.full_well = 10000;     % ~10 ke-
        sp.dark_current_ref = 0.1;
        sp.adc_bits = 12;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 0.80;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.65;
        sp.exposure_time = 0.050;
        sp.analog_gain = 6.0;     % 10ke FW + 12bit, mag0→~50% fill(NF)
        sp.digital_gain = 1.0;

    case 'IMX250'
        % === Sony IMX250 - Pregius 2세대 고속, Global Shutter ===
        % IMX264 고속형 (89fps), 8ch LVDS. 2/3" 포맷.
        sp = struct();
        sp.sensor_name = 'IMX250';
        sp.l = 2448;
        sp.w = 2048;
        sp.myu = 3.45e-6;
        sp.quantum_efficiency = 0.72;
        sp.read_noise = 3.5;
        sp.full_well = 10000;
        sp.dark_current_ref = 0.1;
        sp.adc_bits = 12;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 0.80;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.65;
        sp.exposure_time = 0.050;
        sp.analog_gain = 6.0;     % IMX264 동일 사양, 73%→~55% fill
        sp.digital_gain = 1.0;

    case 'IMX252'
        % === Sony IMX252 - Pregius 2세대 고속 소형, Global Shutter ===
        % IMX250의 1/1.8" 소형 버전 (3.2MP), 119fps.
        sp = struct();
        sp.sensor_name = 'IMX252';
        sp.l = 2048;
        sp.w = 1536;
        sp.myu = 3.45e-6;
        sp.quantum_efficiency = 0.72;
        sp.read_noise = 3.5;
        sp.full_well = 10000;
        sp.dark_current_ref = 0.1;
        sp.adc_bits = 12;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 0.80;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.65;
        sp.exposure_time = 0.050;
        sp.analog_gain = 6.0;     % IMX264 동일 센서군, mag0→~50% fill(NF)
        sp.digital_gain = 1.0;

    % ============================================================
    %  Sony (기타)
    % ============================================================
    case 'IMX296'
        % === Sony IMX296LQR - Global Shutter ===
        % RPi Global Shutter Camera Module
        sp = struct();
        sp.sensor_name = 'IMX296';
        sp.l = 1456;
        sp.w = 1088;
        sp.myu = 3.45e-6;
        sp.quantum_efficiency = 0.65;
        sp.read_noise = 2.2;
        sp.full_well = 10000;
        sp.dark_current_ref = 0.1;
        sp.adc_bits = 10;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 0.85;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.75;
        sp.exposure_time = 0.050;
        sp.analog_gain = 2.0;     % 10bit + 10ke FW + 3.45um → mag0→~62% fill
        sp.digital_gain = 1.0;

    case 'IMX477'
        % === Sony IMX477 - BSI ===
        % RPi HQ Camera. 12.3MP, 1.55um 소형 픽셀.
        sp = struct();
        sp.sensor_name = 'IMX477';
        sp.l = 4056;
        sp.w = 3040;
        sp.myu = 1.55e-6;
        sp.quantum_efficiency = 0.78;
        sp.read_noise = 3.3;
        sp.full_well = 4500;
        sp.dark_current_ref = 0.05;
        sp.adc_bits = 12;
        sp.shutter_type = 'rolling';
        sp.readout_time = 0.032;
        sp.sensitivity_R = 0.88;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.82;
        sp.exposure_time = 0.050;
        sp.analog_gain = 6.0;     % 1.55um 소형 + 4.5ke FW, mag0→~47% fill
        sp.digital_gain = 1.0;

    case 'IMX219'
        % === Sony IMX219 - Rolling Shutter ===
        % RPi Camera V2. 궤도 실적 있음 (GASPACS, Astro Pi).
        sp = struct();
        sp.sensor_name = 'IMX219';
        sp.l = 3280;
        sp.w = 2464;
        sp.myu = 1.12e-6;
        sp.quantum_efficiency = 0.65;
        sp.read_noise = 4.5;
        sp.full_well = 2800;
        sp.dark_current_ref = 0.15;
        sp.adc_bits = 10;
        sp.shutter_type = 'rolling';
        sp.readout_time = 0.033;
        sp.sensitivity_R = 0.82;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.72;
        sp.exposure_time = 0.050;
        sp.analog_gain = 4.0;     % 10bit + 2.8ke FW, mag0→~50% fill
        sp.digital_gain = 1.0;

    % ============================================================
    %  ams-OSRAM (구 CMOSIS) - Global Shutter
    % ============================================================
    case 'CMV4000'
        % === ams-OSRAM CMV4000 - Global Shutter ===
        % 가장 풍부한 COTS 궤도 실적 (XCAM NuSCIS, 다수 CubeSat)
        % CDS 지원, 5.5um 대형 픽셀, 180fps 고속
        sp = struct();
        sp.sensor_name = 'CMV4000';
        sp.l = 2048;
        sp.w = 2048;
        sp.myu = 5.5e-6;          % 5.5 um (대형)
        sp.quantum_efficiency = 0.50;  % ~50% (FSI)
        sp.read_noise = 10.0;     % <13 e- (사양), 일반 ~10 e-
        sp.full_well = 13500;     % 13.5 ke-
        sp.dark_current_ref = 0.5; % FSI, 대형 픽셀 → 높은 다크전류
        sp.adc_bits = 10;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 0.80;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.60;  % FSI: Blue 감도 낮음
        sp.exposure_time = 0.050;
        sp.analog_gain = 2.0;     % 10bit + 대형 5.5um, mag0→~46% fill
        sp.digital_gain = 1.0;

    case 'CMV2000'
        % === ams-OSRAM CMV2000 - Global Shutter ===
        % CMV4000의 2/3" 소형 버전. 궤도 실적 (XCAM C3D, 2014~)
        % 340fps 초고속
        sp = struct();
        sp.sensor_name = 'CMV2000';
        sp.l = 2048;
        sp.w = 1088;
        sp.myu = 5.5e-6;
        sp.quantum_efficiency = 0.50;
        sp.read_noise = 10.0;
        sp.full_well = 13500;
        sp.dark_current_ref = 0.5;
        sp.adc_bits = 10;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 0.80;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.60;
        sp.exposure_time = 0.050;
        sp.analog_gain = 2.0;     % CMV4000 동일 사양, mag0→~43% fill
        sp.digital_gain = 1.0;

    % ============================================================
    %  OmniVision
    % ============================================================
    case 'OV9281'
        % === OmniVision OV9281 - Global Shutter + MIPI ===
        % 1MP 모노크롬. 가장 저렴한 GS + MIPI 센서 ($5~10)
        % 스테레오 비전용, RPi/Jetson 직결
        % 모노크롬 → Bayer 패턴 없음 (sensitivity 모두 1.0)
        sp = struct();
        sp.sensor_name = 'OV9281';
        sp.l = 1280;
        sp.w = 800;
        sp.myu = 3.0e-6;          % 3.0 um
        sp.quantum_efficiency = 0.55;
        sp.read_noise = 5.0;
        sp.full_well = 6000;
        sp.dark_current_ref = 0.2;
        sp.adc_bits = 10;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 1.0;   % 모노크롬: 균일
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 1.0;
        sp.exposure_time = 0.050;
        sp.analog_gain = 2.0;     % 10bit + 낮은 QE(55%), mag0→~43% fill(NF)
        sp.digital_gain = 1.0;

    % ============================================================
    %  onsemi
    % ============================================================
    case 'MT9P031'
        % === onsemi MT9P031 (구 Aptina) - Rolling Shutter ===
        % 5MP, 다수 CubeSat EO 미션 사용. 저가, 풍부한 레퍼런스.
        sp = struct();
        sp.sensor_name = 'MT9P031';
        sp.l = 2592;
        sp.w = 1944;
        sp.myu = 2.2e-6;          % 2.2 um
        sp.quantum_efficiency = 0.50;
        sp.read_noise = 8.0;
        sp.full_well = 6000;
        sp.dark_current_ref = 0.3;
        sp.adc_bits = 12;
        sp.shutter_type = 'rolling';
        sp.readout_time = 0.033;  % ~33ms
        sp.sensitivity_R = 0.80;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.60;  % 구형 FSI
        sp.exposure_time = 0.050;
        sp.analog_gain = 8.0;
        sp.digital_gain = 1.0;

    case 'AR0134'
        % === onsemi AR0134 - Global Shutter ===
        % 1.2MP, -30~+70C 동작 범위. 병렬/HiSPi 인터페이스.
        sp = struct();
        sp.sensor_name = 'AR0134';
        sp.l = 1280;
        sp.w = 960;
        sp.myu = 3.75e-6;         % 3.75 um
        sp.quantum_efficiency = 0.55;
        sp.read_noise = 6.0;
        sp.full_well = 8000;
        sp.dark_current_ref = 0.3;
        sp.adc_bits = 12;
        sp.shutter_type = 'global';
        sp.readout_time = 0;
        sp.sensitivity_R = 0.82;
        sp.sensitivity_G = 1.0;
        sp.sensitivity_B = 0.62;
        sp.exposure_time = 0.050;
        sp.analog_gain = 6.0;     % 12bit + 3.75um, mag0→~39% fill(NF)
        sp.digital_gain = 1.0;

    otherwise
        supported = ['IMX585, IMX678, IMX662, IMX485, IMX462, ', ...
                     'IMX174, IMX249, IMX264, IMX250, IMX252, ', ...
                     'IMX296, IMX477, IMX219, ', ...
                     'CMV4000, CMV2000, OV9281, MT9P031, AR0134'];
        error('알 수 없는 센서: %s\n지원: %s', sensor_name, supported);
end

%% 렌즈 프리셋
lens = get_lens_preset(lens_name);
sp.f = lens.focal_length;
sp.lens_name = lens.name;
sp.f_number = lens.f_number;
sp.lens_mount = lens.mount;

% PSF sigma 계산: Airy disk + 렌즈 수차
%   Airy radius [m] = 1.22 * lambda * f_number
%   lambda = 550nm (V-band)
%   sigma_airy [pixel] = airy_radius / pixel_size
airy_radius = 1.22 * 550e-9 * lens.f_number;
sp.psf_sigma = max(0.8, airy_radius / sp.myu);  % 최소 0.8 pixel

%% 공통 파라미터 (환경/노이즈)
sp.mag_limit = 6.5;
sp.environment = 'space';
sp.sensor_temp = -20;
sp.add_noise = true;
sp.dark_temp_ref = 25;
sp.dark_temp_double = 6.5;
sp.cosmic_ray_rate = 5;
sp.cosmic_ray_energy = 1000;
sp.stray_light = 0;
sp.dsnu = 0.5;
sp.prnu = 0.01;

%% FOV 계산 (정보용)
sp.FOVx = rad2deg(2 * atan((sp.myu * sp.l / 2) / sp.f));
sp.FOVy = rad2deg(2 * atan((sp.myu * sp.w / 2) / sp.f));

%% 사용자 오버라이드 적용
for i = 1:2:length(varargin)
    sp.(varargin{i}) = varargin{i+1};
end

sensor_params = sp;
end

%% ========== 렌즈 프리셋 ==========
function lens = get_lens_preset(lens_name)
% GET_LENS_PRESET 렌즈 사양 반환

switch lower(lens_name)
    case '6mm'
        % Arducam 6mm Wide Angle Lens (CS-Mount Kit LK004)
        lens.name = '6mm f/1.2';
        lens.focal_length = 0.006;   % 6mm [m]
        lens.f_number = 1.2;
        lens.mount = 'CS';

    case '8mm'
        % Arducam 8mm Lens (CS-Mount Kit LK004)
        lens.name = '8mm f/1.2';
        lens.focal_length = 0.008;
        lens.f_number = 1.2;
        lens.mount = 'CS';

    case '12mm'
        % Arducam 12mm Lens (CS-Mount Kit LK004)
        lens.name = '12mm f/1.4';
        lens.focal_length = 0.012;
        lens.f_number = 1.4;
        lens.mount = 'CS';

    case '16mm'
        % Arducam 16mm Lens (CS-Mount Kit LK004)
        lens.name = '16mm f/1.4';
        lens.focal_length = 0.016;
        lens.f_number = 1.4;
        lens.mount = 'CS';

    case '25mm'
        % Arducam 25mm Lens (CS-Mount Kit LK004)
        lens.name = '25mm f/2.0';
        lens.focal_length = 0.025;
        lens.f_number = 2.0;
        lens.mount = 'CS';

    case '8-50mm'
        % Arducam 8-50mm Zoom Lens (C-Mount, LN057)
        % 기본 50mm (최대 줌, 별센서에 적합)
        lens.name = '8-50mm f/1.4 (zoom @50mm)';
        lens.focal_length = 0.050;
        lens.f_number = 1.4;
        lens.mount = 'C';

    case '8-50mm_8'
        % 줌 렌즈 8mm 위치
        lens.name = '8-50mm f/1.4 (zoom @8mm)';
        lens.focal_length = 0.008;
        lens.f_number = 1.4;
        lens.mount = 'C';

    case '8-50mm_25'
        % 줌 렌즈 25mm 위치
        lens.name = '8-50mm f/1.4 (zoom @25mm)';
        lens.focal_length = 0.025;
        lens.f_number = 1.4;
        lens.mount = 'C';

    otherwise
        error('알 수 없는 렌즈: %s\n지원: 6mm, 8mm, 12mm, 16mm, 25mm, 8-50mm', lens_name);
end
end
