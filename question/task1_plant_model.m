%% task1_plant_model.m (課題1: プラントモデル導出)
%  RDS E2023 車輪制御モータの連続時間・離散時間状態空間モデルの導出
%
%  【課題内容】
%  1. 同定モデルパラメータから連続時間モデルの a_m, b_m を導出する．
%  2. 連続時間状態空間行列 Ac, Bc および出力行列 Cc, Dc を定義する．
%  3. 開ループステップ応答をシミュレーションし，モータの挙動を確認する．

clear; clc; close all;

%% ===== Section 1: 定数パラメータ =====
Ts = 0.001;                 % [s] 制御周期 (1 ms)
ENC_IMP = 2048.0;           % [pulse/rev]
ENC_ROM = 4.0;              % 4逓倍
MOT_PLS_PER_1REV = ENC_IMP * ENC_ROM; % = 8192 [pulse/rev]
WHL_PULSESEC_TO_MOT_ANGVEL = 2.0 * pi / MOT_PLS_PER_1REV; % [rad/pulse]
K_enc = 1.0 / WHL_PULSESEC_TO_MOT_ANGVEL;               % ≈ 1303.797 [pulse/rad]
PWM_MAX = 3198;             % 最大PWM指令値

%% ===== Section 2: プラントモデル導出 =====
% RDS 実機同定モデル (_ff_coeff):
%   a1 = -0.9504,  b0 = 0.01039
% 離散時間差分方程式:
%   v_mot[k] = 0.9504 * v_mot[k-1] + 0.01039 * a_pwm[k]
%
% 状態変数: x = [x_enc; v_mot] (位置 [pulse], 角速度 [rad/s])
% 連続時間モデル: dx/dt = Ac * x + Bc * u,  y = Cc * x + Dc

% TODO: 【課題 1-1】連続時間パラメータ a_m, b_m を求める式を完成させなさい．
% ヒント: e^(a_m * Ts) = 0.9504, ゲイン K_c = 0.01039 / (1 - 0.9504) = -b_m / a_m
a_m = ???;  % 目安: ≈ -50.8924 [rad/s]
b_m = ???;  % 目安: ≈ 10.6605 [rad/(s^2 * count)]

% TODO: 【課題 1-2】連続時間状態空間行列 Ac, Bc, 出力行列 Cc, Dc を定義しなさい．
%   d(x_enc)/dt = K_enc * v_mot
%   d(v_mot)/dt = a_m * v_mot + b_m * u
%   y           = x_enc
Ac = [0, ???;
      0, ???];

Bc = [0;
      ???];

Cc = [???, ???];
Dc = 0;

% ZOH (Zero-Order Hold) による厳密離散化: x[k+1] = Ad*x[k] + Bd*u[k]
Ad = [1,  K_enc * (1 - exp(a_m * Ts)) / (-a_m);
      0,  exp(a_m * Ts)];
Bd = [K_enc * (b_m / (-a_m)) * (Ts - (1 - exp(a_m * Ts)) / (-a_m));
      (b_m / (-a_m)) * (1 - exp(a_m * Ts))];

fprintf('=== 課題1: プラントモデル検証 ===\n');
fprintf('  連続時間 Ac:\n'); disp(Ac);
fprintf('  連続時間 Bc:\n'); disp(Bc);
fprintf('  出力行列 Cc: [%.1f, %.1f], Dc: %d\n\n', Cc(1), Cc(2), Dc);
fprintf('  離散時間 Ad:\n'); disp(Ad);
fprintf('  離散時間 Bd:\n'); disp(Bd);

%% ===== Section 3: 開ループステップ応答による確認シミュレーション =====
t_end = 0.2;               % 0.2秒間
N = round(t_end / Ts);
t = (0:N-1) * Ts;

% ステップPWM入力 (t >= 0.02s で PWM=1000)
u_pwm = zeros(1, N);
u_pwm(t >= 0.02) = 1000;

x = zeros(2, 1);
log_v = zeros(1, N);
log_x = zeros(1, N);

for k = 1:N
    % 状態更新
    x = Ad * x + Bd * u_pwm(k);
    
    % 出力 y = Cc * x + Dc
    y = Cc * x + Dc;
    
    log_v(k) = x(2);
    log_x(k) = y;
end

% 理論上の定常角速度 = K_c * PWM
K_c = b_m / (-a_m);
v_ss_theoretical = K_c * 1000;
fprintf('  PWM=1000 に対する理論定常角速度: %.2f rad/s\n', v_ss_theoretical);
fprintf('  シミュレーション最終角速度:       %.2f rad/s\n', log_v(end));

%% ===== Section 4: プロット =====
figure('Name', '課題1: プラントモデル開ループ応答', 'Position', [200, 200, 800, 600]);

subplot(3, 1, 1);
plot(t * 1000, u_pwm, 'm', 'LineWidth', 1.5);
ylabel('PWM [count]'); title('開ループ入力 PWM'); grid on; xlim([0, t_end*1000]);

subplot(3, 1, 2);
plot(t * 1000, log_v, 'b-', 'LineWidth', 1.5); hold on;
yline(v_ss_theoretical, 'r--', sprintf('理論定常値 (%.1f rad/s)', v_ss_theoretical));
ylabel('角速度 [rad/s]'); title('モータ角速度応答 v_{mot}'); legend('応答', '理論定常値', 'Location', 'best'); grid on; xlim([0, t_end*1000]);

subplot(3, 1, 3);
plot(t * 1000, log_x, 'g-', 'LineWidth', 1.5);
ylabel('位置 [pulse]'); xlabel('時間 [ms]'); title('エンコーダ位置 y = x_{enc}'); grid on; xlim([0, t_end*1000]);
