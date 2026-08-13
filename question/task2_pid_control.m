%% task2_pid_control.m (課題2: PIDコントローラと飽和処理)
%  RDS E2023 車輪速度制御の PID フィードバック制御器および飽和処理の実装
%
%  【課題内容】
%  1. 目標速度 y_ref と フィードバック速度 v_fb から偏差 err を計算する．
%  2. PID 制御演算式（P項 + I項 + D項）を実装する．
%  3. 全制御入力 a_pwm_raw を求め，±PWM_MAX で飽和 (Saturation) させる．
%  4. ステップ目標値に対する追従波形と PWM 制御入力を確認する．

clear; clc; close all;

%% ===== Section 1: 定数パラメータ =====
Ts = 0.001;                 % [s] 制御周期 (1 ms)
ENC_IMP = 2048.0;           % [pulse/rev]
ENC_ROM = 4.0;              % 4逓倍
MOT_PLS_PER_1REV = ENC_IMP * ENC_ROM; % = 8192 [pulse/rev]
WHL_PULSESEC_TO_MOT_ANGVEL = 2.0 * pi / MOT_PLS_PER_1REV; % [rad/pulse]
K_enc = 1.0 / WHL_PULSESEC_TO_MOT_ANGVEL;               % ≈ 1303.797 [pulse/rad]
PWM_MAX = 3198;             % 最大PWM指令値

% プラント離散時間モデル (Ad, Bd)
a_m = log(0.9504) / Ts;
b_m = (0.01039 / (1 - 0.9504)) * (-a_m);
Ad = [1,  K_enc * (1 - exp(a_m * Ts)) / (-a_m);
      0,  exp(a_m * Ts)];
Bd = [K_enc * (b_m / (-a_m)) * (Ts - (1 - exp(a_m * Ts)) / (-a_m));
      (b_m / (-a_m)) * (1 - exp(a_m * Ts))];

%% ===== Section 2: 制御器パラメータ =====
% PID ゲイン (RDS control.hpp 準拠)
Kp = 2.0;
Ki = 200.0;
Kd = 0.0;

% FF (Feed-Forward) 制御パラメータ
USE_FF = true;
ff_inv_b0 = 1.0 / 0.01039;
ff_inv_a1 = -0.9504 / 0.01039;
ff_lpf_a1 = -0.432736;
ff_lpf_b0 =  0.283632;
ff_lpf_b1 =  0.283632;

%% ===== Section 3: シミュレーション設定 =====
t_end = 0.6;                % 0.6秒間
N = round(t_end / Ts);
t = (0:N-1) * Ts;

% 目標速度プロファイル [rad/s] (ステップ反転)
v_ref = zeros(1, N);
v_ref(t >= 0.05 & t < 0.3) =  100.0;  % [rad/s]
v_ref(t >= 0.3 & t < 0.5)  = -100.0;  % [rad/s]
v_ref(t >= 0.5)            =    0.0;

%% ===== Section 4: 制御ループ実行 =====
x          = zeros(2, 1);   % 状態 x = [x_enc; v_mot]
i_sum      = 0.0;           % 積分値
e_prev     = 0.0;           % 前回偏差
prev_y_ref = 0.0;          % 前回目標値

ff_u_prev  = 0.0;
ff_lpf_u1  = 0.0;
ff_lpf_y1  = 0.0;

v_fb       = 0.0;           % フィードバック速度

log_v_ref  = zeros(1, N);
log_v_mot  = zeros(1, N);
log_a_pwm  = zeros(1, N);
log_p_term = zeros(1, N);
log_i_term = zeros(1, N);

for k = 1:N
    y_ref = v_ref(k);
    
    % TODO: 【課題 2-1】偏差 err を計算しなさい．(目標速度 y_ref と 現在の v_fb)
    err = ???;
    
    % Anti-windup (目標値の符号反転時に積分リセット)
    if (prev_y_ref * y_ref < 0.0)
        i_sum = 0.0;
    else
        i_sum = i_sum + Ts * err;
    end
    
    % TODO: 【課題 2-2】PID 制御入力 pid_pwm を計算しなさい．
    % (比例項: Kp * err, 積分項: Ki * i_sum, 微分項: Kd * (err - e_prev) / Ts)
    pid_pwm = ???;
    
    prev_y_ref = y_ref;
    e_prev     = err;
    
    % FF (Feed-Forward) 制御項
    if USE_FF
        ff_inv_out = ff_inv_b0 * y_ref + ff_inv_a1 * ff_u_prev;
        ff_u_prev  = y_ref;
        ff_pwm     = ff_lpf_b0 * ff_inv_out + ff_lpf_b1 * ff_lpf_u1 - ff_lpf_a1 * ff_lpf_y1;
        ff_lpf_u1  = ff_inv_out;
        ff_lpf_y1  = ff_pwm;
    else
        ff_pwm = 0.0;
    end
    
    % TODO: 【課題 2-3】全制御入力 a_pwm_raw (PID + FF) を求め，
    %                   ±PWM_MAX (-PWM_MAX <= a_pwm <= PWM_MAX) で飽和処理した a_pwm を作成しなさい．
    a_pwm_raw = ???;
    a_pwm     = ???;
    
    % プラント状態更新
    x = Ad * x + Bd * a_pwm;
    
    % この課題では理想フィードバック (真のモータ速度) を使用
    v_fb = x(2);
    
    % ログ保存
    log_v_ref(k)  = y_ref;
    log_v_mot(k)  = x(2);
    log_a_pwm(k)  = a_pwm;
    log_p_term(k) = Kp * err;
    log_i_term(k) = Ki * i_sum;
end

%% ===== Section 5: プロット =====
figure('Name', '課題2: PID制御と飽和処理', 'Position', [200, 150, 900, 650]);

subplot(3, 1, 1);
plot(t, log_v_ref, 'b--', 'LineWidth', 1.5, 'DisplayName', '目標速度 v_{ref}'); hold on;
plot(t, log_v_mot, 'r-',  'LineWidth', 1.5, 'DisplayName', 'モータ速度 v_{mot}');
ylabel('角速度 [rad/s]'); title('速度追従性能 (PID + FF)'); legend('Location', 'best'); grid on; xlim([0, t_end]);

subplot(3, 1, 2);
plot(t, log_a_pwm, 'm-', 'LineWidth', 1.3, 'DisplayName', '制御入力 a_{pwm}'); hold on;
yline( PWM_MAX, 'k--', 'PWM MAX (+3198)', 'DisplayName', 'PWM上限');
yline(-PWM_MAX, 'k--', 'PWM MIN (-3198)', 'DisplayName', 'PWM下限');
ylabel('PWM [count]'); title('制御入力 a_{pwm} (飽和リミット付き)'); legend('Location', 'best'); grid on; xlim([0, t_end]);

subplot(3, 1, 3);
plot(t, log_p_term, 'g-', 'LineWidth', 1.2, 'DisplayName', 'P項 (Kp * err)'); hold on;
plot(t, log_i_term, 'c-', 'LineWidth', 1.2, 'DisplayName', 'I項 (Ki * \int err)');
ylabel('PWM成分 [count]'); xlabel('時間 [s]'); title('PID 各項の内訳'); legend('Location', 'best'); grid on; xlim([0, t_end]);
