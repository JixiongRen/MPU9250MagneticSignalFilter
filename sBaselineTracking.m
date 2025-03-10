% 参数设置
Fs = 1000;          % 采样率
fc = 0.5;           % 截止频率0.5Hz
duration = 10;       % 信号时长（秒）
t = 0:1/Fs:duration - 1/Fs;
N = length(t);

% 生成含基线漂移的测试信号
signal = 2*sin(2*pi*5*t) + 0.5*sin(2*pi*0.2*t); % 5Hz信号 + 0.2Hz基线漂移
noisy_signal = signal + 0.1*randn(size(t));      % 添加噪声

% 计算滤波器系数
T = 1/Fs;
tau = 1/(2*pi*fc);
alpha = T / (tau + T);

% 初始化基线估计
baseline = zeros(size(noisy_signal));
baseline(1) = noisy_signal(1);

% 迭代滤波
for i = 2:N
    baseline(i) = alpha * noisy_signal(i) + (1 - alpha) * baseline(i-1);
end

% 基线校正
corrected_signal = noisy_signal - baseline;

% 绘图
figure;
subplot(3,1,1);
plot(t, noisy_signal);
title('原始信号');
xlabel('时间 (秒)');

subplot(3,1,2);
plot(t, baseline);
title('基线估计');
xlabel('时间 (秒)');

subplot(3,1,3);
plot(t, corrected_signal);
title('校正后信号');
xlabel('时间 (秒)');