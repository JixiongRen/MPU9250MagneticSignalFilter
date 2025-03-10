clear; clc; close all;

load("StaticMagneticSignalData.mat")

figure;
for i = 1:12
    if isempty(sensorData{i})
        continue; % 跳过没有数据的传感器
    end
    
    % 读取数据
    time = sensorData{i}(:,1); % 时间戳
    xData = sensorData{i}(:,2); % X 轴
    yData = sensorData{i}(:,3); % Y 轴
    zData = sensorData{i}(:,4); % Z 轴
    
    % 绘制子图
    subplot(3, 4, i);
    plot(time, xData, 'r', 'LineWidth', 1.2); hold on;
    plot(time, yData, 'g', 'LineWidth', 1.2);
    plot(time, zData, 'b', 'LineWidth', 1.2);
    
    % 标注
    xlabel('时间 (秒)');
    ylabel('磁场强度');
    title(['传感器 ', num2str(i)]);
    legend('X 轴', 'Y 轴', 'Z 轴');
    grid on;
end

sensorIndex = 4;
% 提取该传感器的数据
sensorDataSingle = sensorData{sensorIndex};
timestamps = sensorDataSingle(:, 1); % 时间戳
x_data = sensorDataSingle(:, 2);    % x轴数据
y_data = sensorDataSingle(:, 3);    % y轴数据
z_data = sensorDataSingle(:, 4);    % z轴数据

% 采样率
fs = 40; % 采样频率 (Hz)

% 使用pwelch计算功率谱密度
window = hann(512); % 选择窗函数，这里使用汉宁窗
noverlap = 256;     % 重叠点数
nfft = 1024;        % FFT点数

% 对x轴数据做功率谱密度估计
[pxx_x, f_x] = pwelch(x_data, window, noverlap, nfft, fs);

% 对y轴数据做功率谱密度估计
[pxx_y, f_y] = pwelch(y_data, window, noverlap, nfft, fs);

% 对z轴数据做功率谱密度估计
[pxx_z, f_z] = pwelch(z_data, window, noverlap, nfft, fs);

% 绘制功率谱密度图
figure;
subplot(3, 1, 1);
plot(f_x, 10*log10(pxx_x)); % 转换为dB单位
title('X轴功率谱密度');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB/Hz)');

subplot(3, 1, 2);
plot(f_y, 10*log10(pxx_y)); % 转换为dB单位
title('Y轴功率谱密度');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB/Hz)');

subplot(3, 1, 3);
plot(f_z, 10*log10(pxx_z)); % 转换为dB单位
title('Z轴功率谱密度');
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB/Hz)');

% 绘制时域波形
figure;
subplot(3, 1, 1);
plot(timestamps, x_data);
title('X轴时域波形');
xlabel('时间戳');
ylabel('幅值');

subplot(3, 1, 2);
plot(timestamps, y_data);
title('Y轴时域波形');
xlabel('时间戳');
ylabel('幅值');

subplot(3, 1, 3);
plot(timestamps, z_data);
title('Z轴时域波形');
xlabel('时间戳');
ylabel('幅值');