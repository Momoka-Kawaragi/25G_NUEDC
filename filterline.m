%% 串口实时幅频/相频特性曲线绘制
clear; clc; close all;

%% 1. 串口配置
portName = 'COM10';      % 串口号
baudRate = 115200;      % 波特率
timeout = 100;           % 超时时间(s)

% 查找并清理已占用的串口
if ~isempty(instrfind('Port', portName))
    fclose(instrfind('Port', portName));
    delete(instrfind('Port', portName));
end

% 创建串口对象 (MATLAB R2019b+ 推荐使用 serialport)
try
    s = serialport(portName, baudRate, 'Timeout', timeout);
    configureTerminator(s, "CR/LF");
    disp(['成功打开 ', portName, '，等待扫频数据...']);
catch ME
    error(['无法打开串口 ', portName, ': ', ME.message]);
end

%% 2. 数据读取与解析逻辑
% 预定义变量
freq_amp = [];
amp_vals = [];
freq_phase = [];
phase_vals = [];

while true
    line = readline(s);
    if isempty(line), continue; end
    
    % 打印收到的原始行进行调试
    % disp(line);
    
    % 捕获幅度数据包 (更新格式: Amplitude Profile (Vpp mV):)
    if contains(line, 'Amplitude Profile')
        raw_data = readline(s); % 读取下一行 "[" 之后的内容
        str = char(raw_data);
        str = strrep(str, '[', '');
        str = strrep(str, ']', '');
        nums = str2num(str); 
        if ~isempty(nums)
            freq_amp = nums(1:2:end);
            amp_vals = nums(2:2:end); % 此时收到的是 Vpp (mV)
            disp(['收到幅度数据点数: ', num2str(length(freq_amp))]);
        end
    end
    
    % 捕获相位数据包 (格式: [频率,相位],[频率,相位]...)
    if contains(line, 'Phase Profile (Degrees):')
        raw_data = readline(s);
        str = char(raw_data);
        str = strrep(str, '[', '');
        str = strrep(str, ']', '');
        nums = str2num(str);
        if ~isempty(nums)
            freq_phase = nums(1:2:end);
            phase_vals = nums(2:2:end);
            disp(['收到相位数据点数: ', num2str(length(freq_phase))]);
            % 收到相位后通常意味着一次完整扫描结束，开始绘图
            break; 
        end
    end
end

% 关闭串口
clear s;

%% 3. 绘图
if isempty(freq_amp) || isempty(freq_phase)
    error('未接收到完整的数据，请检查单片机串口输出格式。');
end

figure('Color','w','Name','扫频特性曲线','Position',[100,100,1000,900]);

% 子图 1: 幅频特性 (实际 Vpp mV)
subplot(3,1,1);
plot(freq_amp, amp_vals, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 3);
grid on; grid minor;
xlabel('频率 (Hz)');
ylabel('Vpp (mV)');
title('1. 幅频特性曲线 (实际 Vpp 数据)');
xlim([min(freq_amp), max(freq_amp)]);

% 子图 2: 幅频特性 (转换为 dB 格式)
subplot(3,1,2);
amp_db = 20*log10(max(amp_vals, 1e-6)); % 防止 log(0)
plot(freq_amp, amp_db, 'g-o', 'LineWidth', 1.5, 'MarkerSize', 3);
grid on; grid minor;
xlabel('频率 (Hz)');
ylabel('幅度 (dB)');
title('2. 幅频特性曲线 (dB 格式)');
xlim([min(freq_amp), max(freq_amp)]);

% 子图 3: 相频特性
subplot(3,1,3);
plot(freq_phase, phase_vals, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 3);
grid on; grid minor;
xlabel('频率 (Hz)');
ylabel('相位 (Deg)');
title('3. 相频特性曲线 (优化翻折与偏置后)');
xlim([min(freq_phase), max(freq_phase)]);

linkaxes(get(gcf,'Children'), 'x'); % 缩放对齐
disp('绘图完成：显示Vpp、dB幅频及相频三部分。');

% 子图 3: 相频特性
subplot(3,1,3);
plot(freq_phase, phase_vals, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 3);
grid on; grid minor;
xlabel('频率 (Hz)');
ylabel('相位 (Deg)');
title('3. 相频特性曲线 (优化翻折与偏置后)');
xlim([min(freq_phase), max(freq_phase)]);

linkaxes(get(gcf,'Children'), 'x'); % 缩放对齐
disp('绘图完成：显示原始ADC、dB幅频及相频三部分。');