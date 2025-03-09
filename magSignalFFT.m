clear;clc;close all;

load("EnvStrayMagSignals.mat")
sensorIndex = 11;

% 提取该传感器的数据
sensorDataSingle = sensorData{sensorIndex}; % 获取第1个传感器的数据
timestamps = sensorDataSingle(:, 1); % 时间戳
x_data = sensorDataSingle(:, 2);    % x轴数据
y_data = sensorDataSingle(:, 3);    % y轴数据
z_data = sensorDataSingle(:, 4);    % z轴数据

% 采样率
fs = 40; % 采样频率 (Hz)

% 对三个轴的数据分别进行FFT
N = length(x_data); % 数据点数
frequencies = (0:N-1) * (fs / N); % 频率轴

% 对x轴数据做FFT
X_fft = fft(x_data);
X_magnitude = abs(X_fft / N); % 计算幅值
X_magnitude = X_magnitude(1:N/2+1); % 取单边频谱
X_magnitude(2:end-1) = 2 * X_magnitude(2:end-1); % 修正幅值

% 对y轴数据做FFT
Y_fft = fft(y_data);
Y_magnitude = abs(Y_fft / N); % 计算幅值
Y_magnitude = Y_magnitude(1:N/2+1); % 取单边频谱
Y_magnitude(2:end-1) = 2 * Y_magnitude(2:end-1); % 修正幅值

% 对z轴数据做FFT
Z_fft = fft(z_data);
Z_magnitude = abs(Z_fft / N); % 计算幅值
Z_magnitude = Z_magnitude(1:N/2+1); % 取单边频谱
Z_magnitude(2:end-1) = 2 * Z_magnitude(2:end-1); % 修正幅值

% 频率轴（单边频谱）
frequencies = frequencies(1:N/2+1);

% 绘制频谱图
figure;
subplot(3, 1, 1);
plot(frequencies, X_magnitude);
title('X轴频谱');
xlabel('频率 (Hz)');
ylabel('幅值');

subplot(3, 1, 2);
plot(frequencies, Y_magnitude);
title('Y轴频谱');
xlabel('频率 (Hz)');
ylabel('幅值');

subplot(3, 1, 3);
plot(frequencies, Z_magnitude);
title('Z轴频谱');
xlabel('频率 (Hz)');
ylabel('幅值');

% 绘制时域波形
figure;
subplot(3, 1, 1);
plot(1:1:length(x_data), x_data);
title('X轴时域波形');
xlabel('采样点');
ylabel('幅值');

subplot(3, 1, 2);
plot(1:1:length(y_data), y_data);
title('Y轴时域波形');
xlabel('采样点');
ylabel('幅值');

subplot(3, 1, 3);
plot(1:1:length(z_data), z_data);
title('Z轴时域波形');
xlabel('采样点');
ylabel('幅值');

