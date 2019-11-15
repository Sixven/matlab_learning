function  [theta_a, theta_b, theta_c]  = myRELS( na, nb, nc, d, u, y, N)
%递推增广最小二乘法
%   na：输出部分的阶次
%   nb：输入部分的阶次
%   nc：噪声部分的阶次
%   d：被控对象的时延
%   u：输入信号
%   y：输出信号
%   K：迭代次数

% 设置初值
len_b = nb + 1;
len = na + len_b + nc;

theta = zeros(len, 1);%B中第一个元素也要加入
P = eye(len) * 10^6;

% 以na=2 nb=1 nc=2 d=3为例
% 构造数据向量的子数组
y_part = zeros(na, 1); % [y(k-1); y(k-2)]
u_part = zeros(len_b+d-1, 1); % [u(k-1); u(k-2); u(k-3); u(k-4)]
%多出d-1用来实现右移功能，且第一个元素能保存当前时刻的u
e_part = zeros(nc, 1); % 噪声估计初值 [e(k-1), e(k-2)]

res = [];
k = 1; %k是迭代/采样次数，从1开始，1对应t=0时刻的初始状态; y、u在零时刻有值
while k <= N
    % 认为t=0(k=1)时刻就是第一个采样点时刻
    fai = [-y_part; u_part(d:end); e_part]; % 数据向量
    y_k = y(k); % 采样
    
    % 计算K(k)
    K = P * fai / (1 + fai' * P * fai); 
    % 计算theta的最小二乘估计
    theta = theta + K * (y_k - fai' * theta); % theta定义时就是列向量
    res = [res; theta'];
    % 更新P
    P = ( eye(len) - K * fai') * P;
    % 计算误差，书上的theta应该为k，fai实际上的表述应该为fai(k)，而不是k-1
    ksai = y_k - fai' * theta;
    
    % 更新a部分，当前的值到下一时刻就是k-1时刻
    for i = length(y_part):-1:2
       y_part(i) = y_part(i-1);
    end
    y_part(1) = y_k;
    % 更新b部分
    for i = length(u_part):-1:2
       u_part(i) = u_part(i-1);
    end
    u_part(1) = u(k);
    % 更新ksai(c)部分
    for i = length(e_part):-1:2
        e_part(i) = e_part(i-1);
    end
    e_part(1) = ksai;

    % 更新迭代次数k
    k = k + 1;
end

theta_a = res(:, 1:na);
theta_b = res(:, na+1:end-(nc-1)-1);
theta_c = res(:, end-(nc-1):end);

end
