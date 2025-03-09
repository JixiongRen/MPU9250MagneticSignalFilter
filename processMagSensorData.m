function processMagSensorData(sensorData, sensorIndex)
% 参数说明：
% sensorData   : 从getMagSignal获取的传感器数据单元数组
% sensorIndex  : 要处理的传感器编号（1-12）

%% 初始化参数
% 基线跟踪参数
windowSize = 100;       % 滑动窗口大小（需小于数据总长度）
% 动态检测参数
activityThreshold = 1e-6;  % 方差阈值，需根据实际噪声调整
activityWindow = 50;       % 动态检测窗口
% 滤波器参数
lpFreq = 10;           % 动态模式低通截止频率 (Hz)
hpFreq = 0.1;          % 静态模式高通截止频率 (Hz)
minSegmentLength = 7;       % 最小处理段长度（必须 > 3*滤波器阶数）

%% 数据准备
% 提取指定传感器数据
data = sensorData{sensorIndex};
timestamps = data(:,1);
x_raw = data(:,2);
y_raw = data(:,3);
z_raw = data(:,4);

% 计算采样率
dt = mean(diff(timestamps));
fs = 1/dt;  % 采样频率

%% 阶段1：基线跟踪
fprintf('计算基线...\n');
baseline_x = movmedian(x_raw, windowSize, 'Endpoints','shrink');
baseline_y = movmedian(y_raw, windowSize, 'Endpoints','shrink');
baseline_z = movmedian(z_raw, windowSize, 'Endpoints','shrink');

x_high = x_raw - baseline_x;
y_high = y_raw - baseline_y;
z_high = z_raw - baseline_z;

%% 阶段2：动态状态检测与分段
fprintf('检测动态状态...\n');
N = length(x_raw);
isDynamic = false(N,1);

% 滑动方差计算
var_x = movvar(x_high, activityWindow);
var_y = movvar(y_high, activityWindow);
var_z = movvar(z_high, activityWindow);

isDynamic = (var_x > activityThreshold) | ...
            (var_y > activityThreshold) | ...
            (var_z > activityThreshold);

% 状态平滑处理
isDynamic = smoothDynamicState(isDynamic, round(0.1*fs));

% 获取连续区段
segments = getContinuousSegments(isDynamic);

%% 阶段3：分段滤波
fprintf('执行分段滤波...\n');

% 设计滤波器
[b_low, a_low] = butter(2, lpFreq/(fs/2), 'low');
[b_high, a_high] = butter(2, hpFreq/(fs/2), 'high');

% 初始化输出
x_filtered = x_raw;
y_filtered = y_raw;
z_filtered = z_raw;

% 处理每个区段
for k = 1:size(segments,1)
    startIdx = segments(k,1);
    endIdx = segments(k,2);
    
    if (endIdx - startIdx) < minSegmentLength
        continue; % 跳过过短区段
    end
    
    if segments(k,3) == 1 % 动态区段
        % 低通滤波原始信号
        x_segment = filtfilt(b_low, a_low, x_raw(startIdx:endIdx));
        y_segment = filtfilt(b_low, a_low, y_raw(startIdx:endIdx));
        z_segment = filtfilt(b_low, a_low, z_raw(startIdx:endIdx));
    else % 静态区段
        % 高通滤波高频分量
        x_segment = filtfilt(b_high, a_high, x_high(startIdx:endIdx));
        y_segment = filtfilt(b_high, a_high, y_high(startIdx:endIdx));
        z_segment = filtfilt(b_high, a_high, z_high(startIdx:endIdx));
    end
    
    % 更新输出
    x_filtered(startIdx:endIdx) = x_segment;
    y_filtered(startIdx:endIdx) = y_segment;
    z_filtered(startIdx:endIdx) = z_segment;
end

%% 可视化结果
figure('Name','磁传感器信号处理结果','NumberTitle','off', 'Position',[100 100 1200 800])

% 原始信号 vs 处理结果
subplot(3,2,1)
plot(timestamps, x_raw)
hold on
plot(timestamps, baseline_x, 'LineWidth',2)
title('X轴原始信号与基线')
legend('原始信号','估计基线')

subplot(3,2,3)
plot(timestamps, y_raw)
hold on
plot(timestamps, baseline_y, 'LineWidth',2)
title('Y轴原始信号与基线')

subplot(3,2,5)
plot(timestamps, z_raw)
hold on
plot(timestamps, baseline_z, 'LineWidth',2)
title('Z轴原始信号与基线')

% 滤波结果
subplot(3,2,2)
plot(timestamps, x_filtered)
title('X轴滤波结果')
ylim([-max(abs(x_filtered)) max(abs(x_filtered))])

subplot(3,2,4)
plot(timestamps, y_filtered)
title('Y轴滤波结果')
ylim([-max(abs(y_filtered)) max(abs(y_filtered))])

subplot(3,2,6)
plot(timestamps, z_filtered)
title('Z轴滤波结果')
ylim([-max(abs(z_filtered)) max(abs(z_filtered))])

%% 频谱分析对比
figure('Name','频谱分析','NumberTitle','off', 'Position',[100 100 1000 800])

% X轴频谱
subplot(3,2,1)
[Pxx,f] = pwelch(x_raw, [],[],[],fs);
semilogx(f, 10*log10(Pxx))
title('X轴原始频谱')
xlabel('频率 (Hz)')

subplot(3,2,2)
[Pxx,f] = pwelch(x_filtered, [],[],[],fs);
semilogx(f, 10*log10(Pxx))
title('X轴滤波后频谱')
xlabel('频率 (Hz)')

% Y轴频谱
subplot(3,2,3)
[Pyy,f] = pwelch(y_raw, [],[],[],fs);
semilogx(f, 10*log10(Pyy))
title('Y轴原始频谱')
xlabel('频率 (Hz)')

subplot(3,2,4)
[Pyy,f] = pwelch(y_filtered, [],[],[],fs);
semilogx(f, 10*log10(Pyy))
title('Y轴滤波后频谱')
xlabel('频率 (Hz)')

% Z轴频谱
subplot(3,2,5)
[Pzz,f] = pwelch(z_raw, [],[],[],fs);
semilogx(f, 10*log10(Pzz))
title('Z轴原始频谱')
xlabel('频率 (Hz)')

subplot(3,2,6)
[Pzz,f] = pwelch(z_filtered, [],[],[],fs);
semilogx(f, 10*log10(Pzz))
title('Z轴滤波后频谱')
xlabel('频率 (Hz)')

end

%% 辅助函数：获取连续区段
function segments = getContinuousSegments(isDynamic)
% 返回格式：[startIdx, endIdx, state]
diffState = diff([0; isDynamic; 0]);
startIdx = find(diffState == 1);
endIdx = find(diffState == -1) - 1;
segments = [startIdx, endIdx, isDynamic(startIdx)];
end

%% 辅助函数：状态平滑处理
function smoothedState = smoothDynamicState(rawState, minDuration)
% 消除短于minDuration的状态切换
N = length(rawState);
smoothedState = rawState;

startIdx = 1;
currentState = rawState(1);

for i = 2:N
    if rawState(i) ~= currentState
        endIdx = i-1;
        duration = endIdx - startIdx + 1;
        
        if duration < minDuration
            smoothedState(startIdx:endIdx) = ~currentState;
        end
        
        startIdx = i;
        currentState = rawState(i);
    end
end

% 处理最后一段
endIdx = N;
duration = endIdx - startIdx + 1;
if duration < minDuration
    smoothedState(startIdx:endIdx) = ~currentState;
end
end