function [processedSignal, isSpike] = kalmanFilter(rawSignal, fs, varargin)
% PROCESSMAGSIGNAL 磁信号尖峰抑制与自适应滤波
% 输入参数：
%   rawSignal : 原始信号向量（N×1）
%   fs       : 采样率 (Hz)
% 可选参数（名称-值对）：
%   SpikeThreshold : 尖峰检测阈值（标准差倍数，默认3）
%   WindowSize     : 滑动窗口大小（样本数，默认21）
%   Q              : 卡尔曼过程噪声协方差（默认1e-5）
%   R              : 卡尔曼观测噪声协方差（默认0.1）
%   MedianFilter   : 中值滤波窗口大小（默认5）
% 输出参数：
%   processedSignal : 处理后的信号
%   isSpike         : 尖峰标记向量

% 参数解析
p = inputParser;
addParameter(p, 'SpikeThreshold', 3, @(x)x>0);
addParameter(p, 'WindowSize', 21, @(x)mod(x,2)==1);
addParameter(p, 'Q', 1e-5, @isnumeric);
addParameter(p, 'R', 0.1, @isnumeric);
addParameter(p, 'MedianFilter', 5, @(x)x>0);
parse(p, varargin{:});
params = p.Results;

% 异常检测
assert(length(rawSignal)>params.WindowSize, ...
    '信号长度必须大于滑动窗口大小');
assert(fs>0, '采样率必须为正数');

%% 处理流程
% 阶段1：中值滤波预处理
smoothed = medfilt1(rawSignal, params.MedianFilter);

% 阶段2：动态尖峰检测
[isSpike, cleaned] = detectSpikes(smoothed, ...
    params.SpikeThreshold, params.WindowSize);

% 阶段3：自适应卡尔曼滤波
kalman = applyKalman(cleaned, fs, params.Q, params.R);

% 阶段4：最终平滑
processedSignal = medfilt1(kalman, 3);

%% 嵌套子函数
    function [isSpike, cleaned] = detectSpikes(signal, th, winSize)
        % 动态阈值计算
        movMean = movmean(signal, winSize);
        movStd = movstd(signal, winSize);
        
        % 尖峰检测
        isSpike = abs(signal - movMean) > th * movStd;
        
        % 三次样条插值修复
        validIdx = find(~isSpike);
        if length(validIdx)<2
            warning('有效数据点不足，跳过尖峰修复');
            cleaned = signal;
            return;
        end
        cleaned = signal;
        cleaned(isSpike) = interp1(validIdx, signal(validIdx), ...
            find(isSpike), 'spline');
    end

    function kalman = applyKalman(signal, fs, Q, R)
        % 卡尔曼滤波参数
        dt = 1/fs;
        A = 1;  % 状态转移矩阵
        H = 1;  % 观测矩阵
        
        % 初始化
        x_est = signal(1);
        P = 1;
        kalman = zeros(size(signal));
        
        % 实时处理
        for k = 1:length(signal)
            % 预测
            x_pred = A * x_est;
            P_pred = A * P * A' + Q;
            
            % 更新
            K = P_pred * H' / (H * P_pred * H' + R);
            x_est = x_pred + K * (signal(k) - H * x_pred);
            P = (1 - K*H) * P_pred;
            
            kalman(k) = x_est;
        end
    end

end