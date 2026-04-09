% 定义传递函数 H(s)
num = 5;  % 分子
den = [10^-8, 3e-4, 1];  % 分母系数

% 创建传递函数对象
H = tf(num, den);

% 画出系统的Bode图（幅频特性）
bode(H);

% 设置图形标题
title('Bode Plot of H(s) = 5 / (10^{-8}s^2 + 3 \times 10^{-4}s + 1)');
grid on;