%% task4_velocity_estimation.m (課題4 解答: 後退差分と2次LPFによる速度推定)
%  RDS E2023 実機エンコーダパルス列からの角速度推定 (後退差分 + 60Hz 2次バターワース LPF)
%
%  【課題内容】
%  1. エンコーダ位置パルス x_enc_meas から後退差分 diff_enc とパルス速度 backward_diff [pulse/s] を計算する．
%  2. 2次バターワース LPF (60Hz) の離散差分方程式を計算し，高周波量子化ノイズを除去する．
%  3. LPF 出力 [pulse/s] をモータ角速度 v_fb [rad/s] に単位変換する．
%  4. 真の角速度 v_mot，差分そのままの粗い速度，LPF 推定速度 v_fb を比較する．

clear; clc; close all;

%% ===== Section 1: 定数パラメータ =====
Ts = 0.001;                 % [s] 制御周期 (1 ms)
ENC_IMP = 2048.0;           % [pulse/rev]
ENC_ROM = 4.0;              % 4逓倍
MOT_PLS_PER_1REV = ENC_IMP * ENC_ROM; % = 8192 [pulse/rev]
WHL_PULSESEC_TO_MOT_ANGVEL = 2.0 * pi / MOT_PLS_PER_1REV; % [rad/pulse]
K_enc = 1.0 / WHL_PULSESEC_TO_MOT_ANGVEL;               % ≈ 1303.797 [pulse/rad]

% 2次バターワース LPF 60Hz (RDS z_transform.hpp 準拠)
lpf_a1 = -1.475480;  
lpf_a2 =  0.586920;  
lpf_b0 =  0.027860;  
lpf_b1 =  0.055720;  
lpf_b2 =  0.027860;  

%% ===== Section 2: シミュレーション入力信号の生成 =====
t_end = 0.5;                % 0.5秒間
N = round(t_end / Ts);
t = (0:N-1) * Ts;

% 模擬的な連続モータ角速度 v_mot (加速 -> 定速 -> 減速)
v_mot_true = zeros(1, N);
for k = 1:N
    if t(k) < 0.1
        v_mot_true(k) = 0.0;
    elseif t(k) < 0.25
        v_mot_true(k) = 50.0 * (1 - cos(pi * (t(k) - 0.1) / 0.15)); % スムーズな加速
    elseif t(k) < 0.4
        v_mot_true(k) = 100.0;
    else
        v_mot_true(k) = 100.0 * (1 + cos(pi * (t(k) - 0.4) / 0.1)) / 2;
    end
end

% 理想的なパルス位置積分 & 実機パルス量子化 (round)
x_enc_ideal = cumsum(v_mot_true * K_enc * Ts);
x_enc_meas  = round(x_enc_ideal);

%% ===== Section 3: 速度推定アルゴリズムの実行 =====
x_enc_prev = 0.0;
lpf_u = zeros(1, 3);        % 入力バッファ: [u[k], u[k-1], u[k-2]]
lpf_y = zeros(1, 3);        % 出力バッファ: [y[k], y[k-1], y[k-2]]

log_raw_vel_rad = zeros(1, N);
log_v_fb        = zeros(1, N);

for k = 1:N
    % 【課題 4-1 解答】後退差分と 1秒あたり換算 (1/Ts = 1000)
    diff_enc      = x_enc_meas(k) - x_enc_prev;
    x_enc_prev    = x_enc_meas(k);
    backward_diff = diff_enc * 1000.0;  % [pulse/s]
    
    % LPFなしの粗い速度 [rad/s] (比較観察用)
    log_raw_vel_rad(k) = backward_diff * WHL_PULSESEC_TO_MOT_ANGVEL;
    
    % 【課題 4-2 解答】2次 LPF の差分方程式
    lpf_u = [backward_diff, lpf_u(1), lpf_u(2)];
    lpf_y_new = lpf_b0 * lpf_u(1) + lpf_b1 * lpf_u(2) + lpf_b2 * lpf_u(3) ...
              - lpf_a1 * lpf_y(1) - lpf_a2 * lpf_y(2);
    lpf_y = [lpf_y_new, lpf_y(1), lpf_y(2)];
    
    % 【課題 4-3 解答】[pulse/s] から [rad/s] への単位変換
    v_fb = WHL_PULSESEC_TO_MOT_ANGVEL * lpf_y_new;
    
    log_v_fb(k) = v_fb;
end

fprintf('=== 課題4: 速度推定とLPF検証 (模範解答) ===\n');
fprintf('  粗い差分速度の最大スパイク値: %.2f rad/s\n', max(log_raw_vel_rad));
fprintf('  LPF推定速度の最大値:          %.2f rad/s (真値最大: %.2f rad/s)\n', max(log_v_fb), max(v_mot_true));

%% ===== Section 4: プロット =====
figure('Name', '課題4: 後退差分と2次LPFによる速度推定 (解答)', 'Position', [150, 150, 950, 700]);

subplot(3, 1, 1);
plot(t, x_enc_meas, 'm-', 'LineWidth', 1.2);
ylabel('パルス [pulse]'); title('エンコーダ観測パルス (量子化済み)'); grid on; xlim([0, t_end]);

subplot(3, 1, 2);
plot(t, log_raw_vel_rad, 'Color', [0.7 0.7 0.7], 'LineWidth', 1.0, 'DisplayName', '後退差分のみ (量子化ノイズ大)'); hold on;
plot(t, v_mot_true, 'g--', 'LineWidth', 2.0, 'DisplayName', '真の角速度 v_{mot}');
ylabel('角速度 [rad/s]'); title('後退差分による速度（量子化スパイクの発生）'); legend('Location', 'best'); grid on; xlim([0, t_end]);

subplot(3, 1, 3);
plot(t, v_mot_true, 'g--', 'LineWidth', 2.0, 'DisplayName', '真の角速度 v_{mot}'); hold on;
plot(t, log_v_fb,   'r-',  'LineWidth', 1.5, 'DisplayName', 'LPF 推定速度 v_{fb} (60Hz LPF)');
ylabel('角速度 [rad/s]'); xlabel('時間 [s]'); title('2次バターワースLPF通過後の推定速度（平滑化・微小遅延）'); legend('Location', 'best'); grid on; xlim([0, t_end]);
