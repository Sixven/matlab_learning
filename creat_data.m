function [u, y] = creat_data(N)
%递推增广最小二乘法
% 产生输入、干扰及输出数据

% L = 150;
L = N;
a=[1.5 0.6];b=[2,-1.4];c=[1,1.2,0.85];%系统输入输出表达式

% 随机数生成器设置
rng('default')
s = rng(0);

e = 0.8*randn(L,1);%精确干扰
u = randn(L,1); % 输入 
y = zeros(L,1); % 输出
seeta_stand = [a,b,c(2:3)];

e0 = zeros(1,2);
u0 = zeros(1,4);
y0 = zeros(1,2);
fa = zeros(1,6);
for i = 1:L
    fa = [-y0,u0(3:4),e0]';
    y(i) = fa'*seeta_stand' + e(i);
    
    for ie = length(e0):-1:2
        e0(ie) = e0(ie-1);
    end
    e0(1) = e(i);
    for iu = length(u0):-1:2
        u0(iu) = u0(iu-1);
    end
    u0(1) = u(i);
    for iy = length(y0):-1:2
        y0(iy) = y0(iy-1);
    end
    y0(1) = y(i);
end

t = 0:0.01:(L-1)*0.01;
%输入、干扰、输出可视化
figure
subplot(211)
plot(t,u,'b',t,e,'g')
legend('u','e')
subplot(212)
plot(t,y)
legend('y')

% 保存数据到mat文件
t = 0:0.01:1.49;
input_source = [t;u'];
ksai_source = [t;e'];
save('input.mat','input_source');
save('ksai.mat', 'ksai_source');

end
