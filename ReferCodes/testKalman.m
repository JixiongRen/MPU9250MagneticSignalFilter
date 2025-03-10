% 生成测试信号
fs = 1000;
t = 0:1/fs:5;
signal = 0.5*sin(2*pi*2*t) + 0.1*randn(size(t));

% 添加尖峰干扰
spikePos = [1000, 2500, 4000];
signal(spikePos) = signal(spikePos) + [3, -2.5, 4];

% 调用处理函数
[processed, isSpike] = kalmanFilter(signal, fs, ...
    'SpikeThreshold', 2.5, ...
    'Q', 1e-4, ...
    'R', 0.2);

% 可视化结果
figure
subplot(211)
plot(t, signal), hold on
plot(t(spikePos), signal(spikePos), 'rx', 'MarkerSize',10)
title('原始信号（红色标记为人工添加尖峰）')
subplot(212)
plot(t, processed), hold on
plot(t(isSpike), processed(isSpike), 'go')
title('处理结果（绿色标记检测到的尖峰）')