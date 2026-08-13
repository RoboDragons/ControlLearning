%% task3_output_equation.m (課題3 解答: 出力方程式とエンコーダ量子化)
%  RDS E2023 状態空間モデルの出力方程式 y = Cx + d とパルス量子化の検証
%
%  【課題内容】
%  1. 状態 x = [x_enc; v_mot] から出力方程式 y = Cx + d を用いてエンコーダ位置を計算する．
%  2. 実機エンコーダのパルス整数化 (round) による量子化ノイズの発生を確認する．
%  3. 量子化誤差（連続値 vs 離散パルス）をプロットして特性を理解する．

clear; clc; close all;

%% ===== Section 1: 定数パラメータ =====
Ts = 0.001;                 % [s] 制御周期 (1 ms)
ENC_IMP = 2048.0;           % [pulse/rev]
ENC_ROM = 4.0;              % 4逓倍
MOT_PLS_PER_1REV = ENC_IMP * ENC_ROM; % = 8192 [pulse/rev]
WHL_PULSESEC_TO_MOT_ANGVEL = 2.0 * pi / MOT_PLS_PER_1REV; % [rad/pulse]
K_enc = 1.0 / WHL_PULSESEC_TO_MOT_ANGVEL;               % ≈ 1303.797 [pulse/rad]

% 状態空間モデル
a_m = log(0.9504) / Ts;
b_m = (0.01039 / (1 - 0.9504)) * (-a_m);
Ad = [1,  K_enc * (1 - exp(a_m * Ts)) / (-a_m);
      0,  exp(a_m * Ts)];
Bd = [K_enc * (b_m / (-a_m)) * (Ts - (1 - exp(a_m * Ts)) / (-a_m));
      (b_m / (-a_m)) * (1 - exp(a_m * Ts))];

Cc = [1, 0];
Dc = 0;

%% ===== Section 2: シミュレーション設定 =====
t_end = 0.1;                % 0.1秒間 (100 ms)
N = round(t_end / Ts);
t = (0:N-1) * Ts;

% 低速〜中速での定速回転を模擬 (PWM入力)
u_pwm = 200 * ones(1, N);   % 一定PWM

%% ===== Section 3: シミュレーション実行 =====
x = zeros(2, 1);
log_x_continuous = zeros(1, N);
log_x_quantized  = zeros(1, N);
log_v_mot        = zeros(1, N);

for k = 1:N
    % 状態更新: x[k+1] = Ad * x[k] + Bd * u[k]
    x = Ad * x + Bd * u_pwm(k);
    
    % 【課題 3 解答】出力方程式 y = Cx + d
    x_enc = Cc * x + Dc;
    
    % 実機エンコーダのパルス整数化 (量子化)
    x_enc_meas = round(x_enc);
    
    log_x_continuous(k) = x_enc;
    log_x_quantized(k)  = x_enc_meas;
    log_v_mot(k)        = x(2);
end

% 量子化誤差の計算
quantization_error = log_x_quantized - log_x_continuous;

fprintf('=== 課題3: 出力方程式と量子化誤差 (模範解答) ===\n');
fprintf('  最大量子化誤差: %.4f [pulse] (理論値: ±0.5 pulse)\n', max(abs(quantization_error)));
fprintf('  最終連続位置:   %.3f [pulse]\n', log_x_continuous(end));
fprintf('  最終離散パルス: %d [pulse]\n', log_x_quantized(end));

%% ===== Section 4: プロット =====
figure('Name', '課題3: 出力方程式とエンコーダパルス量子化 (解答)', 'Position', [200, 150, 900, 650]);

subplot(3, 1, 1);
plot(t * 1000, log_v_mot, 'b-', 'LineWidth', 1.5);
ylabel('角速度 [rad/s]'); title('モータ真の角速度 v_{mot}'); grid on; xlim([0, t_end*1000]);

subplot(3, 1, 2);
plot(t * 1000, log_x_continuous, 'g-', 'LineWidth', 1.5, 'DisplayName', '連続値 y = Cx + d'); hold on;
stairs(t * 1000, log_x_quantized, 'r--', 'LineWidth', 1.2, 'DisplayName', '量子化パルス round(y)');
ylabel('位置 [pulse]'); title('エンコーダ位置出力'); legend('Location', 'northwest'); grid on; xlim([0, t_end*1000]);

subplot(3, 1, 3);
stem(t * 1000, quantization_error, 'm', 'MarkerSize', 4);
ylabel('誤差 [pulse]'); xlabel('時間 [ms]'); title('パルス量子化誤差 (x_{meas} - x_{real})'); grid on; xlim([0, t_end*1000]);
ylim([-0.7, 0.7]);
