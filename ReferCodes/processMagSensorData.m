function [x_filtered, y_filtered, z_filtered]=processMagSensorData(sensorData, sensorIndex)
% 参数说明：
% sensorData   : 从getMagSignal获取的传感器数据单元数组
% sensorIndex  : 要处理的传感器编号（1-12）

%% 初始化参数
windowSize = 100;           % 基线跟踪窗口
activityThreshold = 1e-6;   % 动态检测方差阈值
activityWindow = 50;        % 动态检测窗口
lpFreq = 10;                % 低通截止频率 (Hz)
hpFreq = 0.1;               % 高通截止频率 (Hz)
minSegmentLength = 7;       % 最小处理段长度

% 小波去噪参数
waveletName = 'sym4';       % 使用的小波基
decompLevel = 5;            % 分解层数
denoiseMethod = 'sqtwolog'; % 阈值方法

%% 数据准备
data = sensorData{sensorIndex};
timestamps = data(:,1);
x_raw = data(:,2);
y_raw = data(:,3);
z_raw = data(:,4);

dt = mean(diff(timestamps));
fs = 1/dt;

%% 阶段1：基线跟踪
fprintf('计算基线...\n');
baseline_x = movmedian(x_raw, windowSize, 'Endpoints','shrink');
baseline_y = movmedian(y_raw, windowSize, 'Endpoints','shrink');
baseline_z = movmedian(z_raw, windowSize, 'Endpoints','shrink');

x_high = x_raw - baseline_x;
y_high = y_raw - baseline_y;
z_high = z_raw - baseline_z;

%% 阶段1.5：小波去噪
fprintf('执行小波去噪...\n');
x_high = wden(x_high, denoiseMethod, 'h', 'mln', decompLevel, waveletName);
y_high = wden(y_high, denoiseMethod, 'h', 'mln', decompLevel, waveletName);
z_high = wden(z_high, denoiseMethod, 'h', 'mln', decompLevel, waveletName);

%% 阶段2：动态状态检测与分段
fprintf('检测动态状态...\n');
N = length(x_raw);
var_x = movvar(x_high, activityWindow);
var_y = movvar(y_high, activityWindow);
var_z = movvar(z_high, activityWindow);

isDynamic = (var_x > activityThreshold) | ...
            (var_y > activityThreshold) | ...
            (var_z > activityThreshold);

isDynamic = smoothDynamicState(isDynamic, round(0.1*fs));
segments = getContinuousSegments(isDynamic);

%% 阶段3：分段滤波
fprintf('执行分段滤波...\n');
[b_low, a_low] = butter(2, lpFreq/(fs/2), 'low');
[b_high, a_high] = butter(2, hpFreq/(fs/2), 'high');

x_filtered = x_raw;
y_filtered = y_raw;
z_filtered = z_raw;

for k = 1:size(segments,1)
    startIdx = segments(k,1);
    endIdx = segments(k,2);
    
    if (endIdx - startIdx) < minSegmentLength
        continue;
    end
    
    if segments(k,3) == 1
        x_segment = filtfilt(b_low, a_low, x_raw(startIdx:endIdx));
        y_segment = filtfilt(b_low, a_low, y_raw(startIdx:endIdx));
        z_segment = filtfilt(b_low, a_low, z_raw(startIdx:endIdx));
    else
        x_segment = filtfilt(b_high, a_high, x_high(startIdx:endIdx));
        y_segment = filtfilt(b_high, a_high, y_high(startIdx:endIdx));
        z_segment = filtfilt(b_high, a_high, z_high(startIdx:endIdx));
    end
    
    x_filtered(startIdx:endIdx) = x_segment;
    y_filtered(startIdx:endIdx) = y_segment;
    z_filtered(startIdx:endIdx) = z_segment;
end

%% 增强可视化
plotRange = @(data) [-1.1*max(abs(data)), 1.1*max(abs(data))]; % 统一显示范围函数

% 时域对比
figure('Name','时域分析','NumberTitle','off', 'Position',[100 100 1400 900])
plotTDRange(x_raw, x_filtered, timestamps, 'X轴', 1, plotRange([x_raw; x_filtered]))
plotTDRange(y_raw, y_filtered, timestamps, 'Y轴', 3, plotRange([y_raw; y_filtered]))
plotTDRange(z_raw, z_filtered, timestamps, 'Z轴', 5, plotRange([z_raw; z_filtered]))

% 频域对比
figure('Name','频域分析','NumberTitle','off', 'Position',[100 100 1400 900])
plotPSDRange(x_raw, x_filtered, fs, 'X轴', 1)
plotPSDRange(y_raw, y_filtered, fs, 'Y轴', 3)
plotPSDRange(z_raw, z_filtered, fs, 'Z轴', 5)

end

%% 可视化子函数
function plotTDRange(raw, filtered, t, titleStr, subPos, yRange)
subplot(3,2,subPos)
plot(t, raw)
hold on
plot(t, filtered, 'LineWidth',1.5)
title([titleStr '时域信号'])
xlabel('时间 (s)'), ylabel('幅值')
grid on
ylim(yRange)
legend('原始信号','处理后')

subplot(3,2,subPos+1)
plot(t, filtered)
title([titleStr '处理后细节'])
xlabel('时间 (s)'), ylabel('幅值')
grid on
ylim(yRange)
end

function plotPSDRange(raw, filtered, fs, titleStr, subPos)
subplot(3,2,subPos)
[Pxx, f] = pwelch(raw, hanning(1024), 512, 1024, fs);
semilogx(f, 10*log10(Pxx))
hold on
[PxxF, f] = pwelch(filtered, hanning(1024), 512, 1024, fs);
semilogx(f, 10*log10(PxxF))
title([titleStr '功率谱'])
xlabel('频率 (Hz)'), ylabel('PSD (dB/Hz)')
grid on
xlim([1e-2 fs/2])
legend('原始','处理后')

subplot(3,2,subPos+1)
plot(f, 10*log10(PxxF./Pxx))
title([titleStr 'PSD变化'])
xlabel('频率 (Hz)'), ylabel('增益 (dB)')
grid on
xlim([1e-2 fs/2])
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