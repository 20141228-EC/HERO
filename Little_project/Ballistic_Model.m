% 高尔夫球轨迹模拟，考虑空气阻力，落点高于发射点 0.2m，筛选距离范围
clc;
clear;

% 参数设置
v0 = 15.8; % 初速度，单位m/s
angles = 20:0.1:80; % 发射角度范围，单位度
g = 9.81; % 重力加速度，单位m/s^2
rho = 1.225; % 空气密度，单位kg/m^3
Cd = 0.58; % 阻力系数
A = 0.001432; % 球的横截面积，单位m^2
m = 0.04593; % 球的质量，单位kg
target_height = 1; % 落点高于发射点的高度差，单位m

% 结果存储
distances = zeros(size(angles)); % 用于存储不同角度的落点距离
 
% 循环计算每个发射角度的轨迹
for i = 1:length(angles)
    theta = deg2rad(angles(i)); % 转为弧度
    % 初始条件
    vx = v0 * cos(theta); % 水平速度分量
    vy = v0 * sin(theta); % 垂直速度分量
    x = 0; % 初始水平位置
    y = 0; % 初始垂直位置
    
    % 时间步长和初始时间
    dt = 0.001; % 时间步长
    t = 0;
    
    % 模拟轨迹
    while y >= 0 || (y < 0 && y + vy * dt >= target_height)
        % 空气阻力
        v = sqrt(vx^2 + vy^2); % 瞬时速度
        ax = - (1/2) * rho * Cd * A * v * vx / m; % 水平加速度
        ay = -g - (1/2) * rho * Cd * A * v * vy / m; % 垂直加速度
        
        % 更新速度
        vx = vx + ax * dt;
        vy = vy + ay * dt;
        
        % 更新位置
        x_prev = x; % 记录前一时刻位置
        y_prev = y;
        x = x + vx * dt;
        y = y + vy * dt;
        
        % 检查是否达到目标高度
        if y_prev >= target_height && y < target_height
            % 线性插值计算精确的水平位置
            slope = (y - y_prev) / (x - x_prev);
            x = x_prev + (target_height - y_prev) / slope;
            break;
        end
        
        % 更新时间
        t = t + dt;
    end
    
    % 记录落地点
    distances(i) = x;
end

% 筛选落点距离在10到25m之间的数据
valid_indices = (distances >= 10) & (distances <= 25);
valid_angles = angles(valid_indices); % 筛选符合条件的角度
valid_distances = distances(valid_indices); % 筛选符合条件的距离

% 拟合函数
p = polyfit(valid_angles, valid_distances, 2); % 二次多项式拟合
fitted_distances = polyval(p, valid_angles);

% 绘制结果
figure;
plot(valid_angles, valid_distances, 'o');
hold on;
plot(valid_angles, fitted_distances, '--r', 'LineWidth', 1.5);
title('10到25m之间的发射角度与落点距离关系');
xlabel('发射角度（度）');
ylabel('落点距离（米）');
legend('筛选数据', '拟合曲线');
grid on;
hold off;

% 输出拟合函数
disp('拟合函数：');
fprintf('y = %.4f*x^2 + %.4f*x + %.4f\n', p(1), p(2), p(3));

% 定义变量 y
syms y; % y 是输入变量
a = p(1); % 二次项系数
b = p(2); % 一次项系数
c = p(3); % 常数项

% 将二次方程改写为 a*x^2 + b*x + (c - y) = 0
solution_positive = (-b + sqrt(b^2 - 4*a*(c - y))) / (2*a); % 正解
solution_negative = (-b - sqrt(b^2 - 4*a*(c - y))) / (2*a); % 负解

disp('x 的解为：');
fprintf(' smaller_angle= (-%.4f + sqrt(%.4f^2 - 4*%.4f*(%.4f - y)))/(2*%.4f)\n', ...
    b, b, a, c, a);
fprintf(' higher_angle = (-%.4f - sqrt(%.4f^2 - 4*%.4f*(%.4f - y)))/(2*%.4f)\n', ...
    b, b, a, c, a);

% 自动提取拟合函数系数
a = p(1); % 二次项系数
b = p(2); % 一次项系数
c = p(3); % 常数项

% 定义目标距离变量
y =20; % 目标距离（可修改）

% 计算判别式
discriminant = b^2 - 4 * a * (c - y);

% 判断是否有实数解
if discriminant >= 0
    % 计算较小根 x2
    x1 = (-b + sqrt(discriminant)) / (2 * a);
    fprintf('拟合函数：y = %.4f*x^2 + %.4f*x + %.4f\n', a, b, c);
    fprintf('当距离 y = %.2f 时，解 x1 = %.4f\n', y, x1);
else
    % 无实数解
    disp('无实数解，判别式小于 0');
end