clear;clc;close all;
% 示例：处理第1个传感器的数据
load('EnvStrayMagSignals.mat'); % 加载数据
[x_filtered, y_filtered, z_filtered] = processMagSensorData(sensorData, 11);

fs = 40;
t = (1:1:25826)/fs;

% 调用处理函数
[processed, isSpike] = kalmanFilter(x_filtered, fs, ...
    'SpikeThreshold', 2.5, ...
    'Q', 1e-4, ...
    'R', 0.2);

% 可视化结果
figure
subplot(211)
plot(t, x_filtered), hold on
title('原始信号')
subplot(212)
plot(t, processed), hold on
title('处理结果')