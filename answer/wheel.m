%% wheel.m
%  車輪速度制御シミュレーション（RDS E2023 忠実準拠モデル）
%
%  【制御構造】
%    v_ref [rad/s]
%        ↓
%     コントローラ (PID + オプションFF制御)
%        ↓ a_pwm (飽和: ±3198)
%     連続状態空間モデル x' = Ax + Bu  (x = [x_enc; v_mot])
%     ※ A, B は RDS 内のモータ同定モデル ff_coeff (1次遅れ) より導出
%        ↓
%     出力方程式 y = Cx + d  (C = [1, 0], d = 0 → y = x_enc [pulse])
%        ↓
%     後退差分 (diff * 1000 [pulse/s])  ※エンコーダパルス量子化ノイズ対応
%        ↓
%     2次バターワースLPF (60Hz, z_transform.hpp 準拠)
%        ↓
%     単位変換 (WHL_PULSESEC_TO_MOT_ANGVEL) → v_fb [rad/s] → FB

clear; clc; close all;

%% ===== Section 1: 定数パラメータ =========================================

% --- 制御オプション ---
USE_QUANTIZATION = true;   % エンコーダのパルス整数化ノイズを再現する場合は true
USE_FF           = true;   % RDS 実機の FF (Feed-Forward) 制御を有効にする場合は true
SHOW_ANIMATION   = true;   % 車輪の回転アニメーションを表示する場合は true

% --- 制御周期 ---
Ts = 0.001;            % [s]  制御周期 (1 ms)

% --- エンコーダパラメータ ---
ENC_IMP          = 2048.0;                              % [pulse/rev]
ENC_ROM          = 4.0;                                 % [-]
MOT_PLS_PER_1REV = ENC_IMP * ENC_ROM;                   % = 8192 [pulse/rev(motor)]

% パルス/s から モータ角速度 [rad/s] への変換係数
WHL_PULSESEC_TO_MOT_ANGVEL = 2.0 * pi / MOT_PLS_PER_1REV; % [rad/pulse]

% エンコーダ変換定数 K_enc [pulse/rad]
K_enc = 1.0 / WHL_PULSESEC_TO_MOT_ANGVEL;               % = 8192 / (2*pi) ≈ 1303.797

% --- PWM 制限値 ---
PWM_MAX = 3198;                                         % [count]

%% ===== Section 2: プラントモデル導出 (x' = Ax + Bu, y = Cx + d) ===========
% RDS コードの同定モデル (_ff_coeff):
%   a1 = -0.9504,  b0 = 0.01039
% 離散時間差分方程式:
%   v_mot[k] = 0.9504 * v_mot[k-1] + 0.01039 * a_pwm[k]
%
% 状態変数 x = [x_enc; v_mot]  (x_enc: 位置 [pulse], v_mot: 角速度 [rad/s])
% 連続時間モデル dx/dt = Ax + Bu:
%   d(x_enc)/dt = K_enc * v_mot
%   d(v_mot)/dt = a_m * v_mot + b_m * u
%
%   a_m = ln(0.9504) / Ts ≈ -50.8924 [rad/s]   (時定数 τ ≈ 19.65 ms)
%   b_m = (0.01039 / (1 - 0.9504)) * (-a_m) ≈ 10.6605 [rad/(s^2 * count)]

a_m = log(0.9504) / Ts;
b_m = (0.01039 / (1 - 0.9504)) * (-a_m);

% 連続時間状態空間行列
Ac = [0, K_enc;
      0,   a_m];
Bc = [0;
      b_m];

% 出力方程式 y = Cx + d  (y = x_enc [pulse])
Cc = [1, 0];
Dc = 0;

% ZOH (Zero-Order Hold) による厳密離散化: x[k+1] = Ad*x[k] + Bd*u[k]
Ad = [1,  K_enc * (1 - exp(a_m * Ts)) / (-a_m);
      0,  exp(a_m * Ts)];
Bd = [K_enc * (b_m / (-a_m)) * (Ts - (1 - exp(a_m * Ts)) / (-a_m));
      (b_m / (-a_m)) * (1 - exp(a_m * Ts))];

fprintf('=== RDS モータプラントモデル (x = [x_enc; v_mot], y = Cx + d) ===\n');
fprintf('  連続時間 A = [0 %.3f; 0 %.4f], B = [0; %.4f]\n', Ac(1,2), Ac(2,2), Bc(2));
fprintf('  離散時間 Ad = [1 %.6f; 0 %.4f], Bd = [%.8f; %.5f]\n', Ad(1,2), Ad(2,2), Bd(1), Bd(2));
fprintf('  出力行列 C = [%d %d], d = %d\n', Cc(1), Cc(2), Dc);

%% ===== Section 3: PID ゲイン, FF フィルタ & LPF 係数 =======================

% --- PID ゲイン (control.hpp より) ---
Kp = 2.0;
Ki = 200.0;
Kd = 0.0;

% --- RDS FF (Feed-Forward) フィルタパラメータ ---
ff_inv_b0 = 1.0 / 0.01039;
ff_inv_a1 = -0.9504 / 0.01039;
ff_lpf_a1 = -0.432736;
ff_lpf_b0 =  0.283632;
ff_lpf_b1 =  0.283632;

% --- 2次バターワース LPF 60Hz (z_transform.hpp 準拠) ---
lpf_a1 = -1.475480;  
lpf_a2 =  0.586920;  
lpf_b0 =  0.027860;  
lpf_b1 =  0.055720;  
lpf_b2 =  0.027860;  

%% ===== Section 4: シミュレーション設定 ====================================

t_end = 1.0;              % [s]  シミュレーション時間
N     = round(t_end / Ts);% ステップ数
t     = (0:N-1) * Ts;     % [s]  時刻ベクトル

% 目標速度 v_ref プロファイル [rad/s]
v_ref = zeros(1, N);
v_ref(t >= 0.1 & t < 0.5) =  100.0;   % [rad/s]
v_ref(t >= 0.5 & t < 0.8) = -50.0;   % [rad/s]
v_ref(t >= 0.8)            =    0.0;

%% ===== Section 5: シミュレーション実行 ====================================

x          = zeros(2, 1);  % 状態 x = [x_enc; v_mot]
i_sum      = 0.0;          % 積分値
e_prev     = 0.0;          % 前回の偏差
prev_y_ref = 0.0;         % 前回の目標値

% FF フィルタバッファ
ff_u_prev   = 0.0;        % FF 逆フィルタ入力履歴
ff_lpf_u1   = 0.0;        % FF LPF 入力履歴
ff_lpf_y1   = 0.0;        % FF LPF 出力履歴

% 速度推定 LPF 履歴バッファ
lpf_u = zeros(1, 3);      % u[k], u[k-1], u[k-2] (入力: pulse/s)
lpf_y = zeros(1, 3);      % y[k], y[k-1], y[k-2] (出力: pulse/s)

x_enc_prev = 0.0;         % 前回のエンコーダ観測値
v_fb       = 0.0;         % 初期 FB 角速度

% ログ用配列
log_v_ref = zeros(1, N);
log_v_fb  = zeros(1, N);
log_v_mot = zeros(1, N);
log_a_pwm = zeros(1, N);
log_x_enc = zeros(1, N);

for k = 1:N
    % (1) 目標速度の取得
    y_ref = v_ref(k);
    
    % (2) PID コントローラ
    err = y_ref - v_fb;
    
    % Anti-windup: 目標値の符号反転時に積分リセット
    if (prev_y_ref * y_ref < 0.0)
        i_sum = 0.0;
    else
        i_sum = i_sum + Ts * err;
    end
    
    pid_pwm = Kp * err + Ki * i_sum + Kd * (err - e_prev) / Ts;
    prev_y_ref = y_ref;
    e_prev     = err;
    
    % (2-b) FF (Feed-Forward) 制御項の計算
    if USE_FF
        ff_inv_out = ff_inv_b0 * y_ref + ff_inv_a1 * ff_u_prev;
        ff_u_prev  = y_ref;
        
        ff_pwm = ff_lpf_b0 * ff_inv_out + ff_lpf_b1 * ff_lpf_u1 - ff_lpf_a1 * ff_lpf_y1;
        ff_lpf_u1 = ff_inv_out;
        ff_lpf_y1 = ff_pwm;
    else
        ff_pwm = 0.0;
    end
    
    % 全制御入力と飽和処理 (±PWM_MAX)
    a_pwm_raw = pid_pwm + ff_pwm;
    a_pwm     = max(-PWM_MAX, min(PWM_MAX, a_pwm_raw));
    
    % (3) プラント更新: x[k+1] = Ad * x[k] + Bd * a_pwm
    x = Ad * x + Bd * a_pwm;
    
    % (4) 出力方程式: y = Cx + d  (y = x_enc [pulse])
    x_enc = Cc * x + Dc;
    
    % パルス量子化（実機エンコーダ離散パルスの再現）
    if USE_QUANTIZATION
        x_enc_meas = round(x_enc);
    else
        x_enc_meas = x_enc;
    end
    
    % (5) 微分器 + 60Hz 2次 LPF (PwmOntimeCalculator::CalcMeas 忠実再現)
    diff_enc   = x_enc_meas - x_enc_prev;
    x_enc_prev = x_enc_meas;
    backward_diff = diff_enc * 1000.0;  % [pulse/s]
    
    % 2次 LPF
    lpf_u = [backward_diff, lpf_u(1), lpf_u(2)];
    lpf_y_new = lpf_b0 * lpf_u(1) + lpf_b1 * lpf_u(2) + lpf_b2 * lpf_u(3) ...
              - lpf_a1 * lpf_y(1) - lpf_a2 * lpf_y(2);
    lpf_y = [lpf_y_new, lpf_y(1), lpf_y(2)];
    
    % [pulse/s] → [rad/s] 単位変換
    v_fb = WHL_PULSESEC_TO_MOT_ANGVEL * lpf_y_new;
    
    % (6) データ保存
    log_v_ref(k) = y_ref;
    log_v_fb(k)  = v_fb;
    log_v_mot(k) = x(2);      % プラント真の速度
    log_a_pwm(k) = a_pwm;
    log_x_enc(k) = x_enc_meas;
end

%% ===== Section 6: 結果プロット (メインウィンドウ) ========================

figWave = figure('Name', 'RDS E2023 車輪速度制御モデル (メイン波形)', 'Position', [150, 100, 900, 700]);

% 1. 速度追従 (v_ref vs v_fb vs v_mot)
ax1 = subplot(3, 1, 1);
plot(t, log_v_ref, 'b--', 'LineWidth', 1.5, 'DisplayName', 'v\_ref [rad/s]'); hold on;
plot(t, log_v_mot, 'g:',  'LineWidth', 1.5, 'DisplayName', 'v\_mot (真値)');
plot(t, log_v_fb,  'r-',  'LineWidth', 1.2, 'DisplayName', 'v\_fb (推定FB値)');
ylabel('角速度 [rad/s]');
title('速度追従性能 (v\_ref vs v\_fb)');
legend('Location', 'best');
grid on; xlim([0, t_end]);

% 2. 制御入力 (a_pwm)
ax2 = subplot(3, 1, 2);
plot(t, log_a_pwm, 'm-', 'LineWidth', 1.2);
yline( PWM_MAX, 'k--', 'PWM MAX (+3198)');
yline(-PWM_MAX, 'k--', 'PWM MIN (-3198)');
ylabel('制御入力 a\_pwm');
title('制御入力 a\_pwm [count] (PID + FF)');
grid on; xlim([0, t_end]);

% 3. エンコーダ出力 (x_enc)
ax3 = subplot(3, 1, 3);
plot(t, log_x_enc, 'c-', 'LineWidth', 1.2);
ylabel('エンコーダ [pulse]');
xlabel('時間 [s]');
title('エンコーダ累積パルス y = Cx + d (x\_enc)');
grid on; xlim([0, t_end]);

linkaxes([ax1, ax2, ax3], 'x');
sgtitle('RDS E2023 車輪モデル シミュレーション結果', 'FontSize', 14, 'FontWeight', 'bold');

%% ===== Section 7: 車輪回転ビジュアルアニメーション (独立ウィンドウ) =======

if SHOW_ANIMATION
    fprintf('\n車輪回転アニメーションを開始します...\n');
    figAnim = figure('Name', '車輪回転ビジュアルアニメーション', 'Position', [1100, 300, 400, 400]);
    theta_arr = log_x_enc * WHL_PULSESEC_TO_MOT_ANGVEL;
    r = 1.0;
    
    for k = 1:10:N
        if ~ishandle(figAnim), break; end % ウィンドウが閉じられたら終了
        figure(figAnim); clf(figAnim);
        theta = theta_arr(k);
        th = stroke_circle(0, 0, r); hold on;
        fill(th(1,:), th(2,:), [0.15 0.2 0.3]);
        plot([0, r*cos(theta)], [0, r*sin(theta)], 'r-', 'LineWidth', 3);
        plot([0, r*cos(theta+pi/2)], [0, r*sin(theta+pi/2)], 'w-', 'LineWidth', 1.5);
        plot([0, r*cos(theta+pi)], [0, r*sin(theta+pi)], 'w-', 'LineWidth', 1.5);
        plot([0, r*cos(theta-pi/2)], [0, r*sin(theta-pi/2)], 'w-', 'LineWidth', 1.5);
        plot(0, 0, 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
        axis equal; axis([-1.3 1.3 -1.3 1.3]); grid on;
        title(sprintf('t = %.3f s | v_{fb} = %.1f rad/s', t(k), log_v_fb(k)));
        drawnow;
    end
end

fprintf('\nシミュレーション終了\n');

function circ = stroke_circle(cx, cy, r)
    t_c = linspace(0, 2*pi, 100);
    circ = [cx + r*cos(t_c); cy + r*sin(t_c)];
end