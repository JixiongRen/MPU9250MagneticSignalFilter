function sensorData = getMagSignal(serialPort, baudRate)
    % 创建串口对象并配置参数
    s = serialport(serialPort, baudRate);
    s.DataBits = 8;
    s.StopBits = 1;
    s.Parity = 'none';
    
    % 初始化存储数据的单元数组
    numSensors = 12;
    numSamples = 100000;
    sensorData = cell(numSensors, 1);
    for i = 1:numSensors
        sensorData{i} = zeros(numSamples, 4); % 预分配内存 [时间戳, x, y, z]
    end
    
    disp('开始接收数据...');
    startTime = tic; % 记录采集开始时间
    
    for k = 1:numSamples
        % 确保读取完整的数据包（144字节）
        dataBytes = read(s, 36*4, 'uint8');
        
        % 校验数据长度
        while numel(dataBytes) ~= 36*4
            warning('数据包长度异常，尝试重新读取...');
            dataBytes = read(s, 36*4, 'uint8');
        end
        
        % 转换字节数据到单精度浮点数
        dataFloats = typecast(uint8(dataBytes), 'single');
        
        % 提取时间戳并填充数据
        timestamp = toc(startTime);
        for i = 1:numSensors
            idx = 3*(i-1) + 1; % 每个传感器数据起始索引
            x = dataFloats(idx);
            y = dataFloats(idx+1);
            z = dataFloats(idx+2);
            sensorData{i}(k, :) = [timestamp, x, y, z];
        end
        
        % 显示进度（每5000组更新一次）
        if mod(k, 5000) == 0
            fprintf('进度: %d/%d 组\n', k, numSamples);
        end
    end
    
    % 关闭并清理串口对象
    delete(s);
    clear s;
    disp('数据采集完成。');
end