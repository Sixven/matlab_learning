

N = 150; %迭代次数

% 创建数据
[u, y] = creat_data(N);

na = 2;nb = 1;nc = 2; % 阶次
d = 3; % 输入的时延

[theta_a_cn_1, theta_b_cn_1, theta_c_cn_1] = myRELS(na, nb, nc, d, u, y, N);
Nt = length(theta_a_cn_1);
% 绘图
A = [1, 1.5, 0.6];
B = [2, -1.4];
C = [1, 1.2, 0.85];
myplot(A, B, C, Nt, theta_a_cn_1, theta_b_cn_1, theta_c_cn_1)
