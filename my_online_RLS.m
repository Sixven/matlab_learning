function [sys,x0,str,ts,simStateCompliance] = my_online_RLS(t,x,u,flag,na,nb,nc,d,alpha)
% 在线递推最小二乘法，可以考虑白噪声和有色噪声
% u ：输入，包括系统的输入和输出
% na：差分方程的A的阶次
% nb：差分方程的B的阶次
% nc：差分方程的C的阶次
% d ：差分方程输入的时延
% alpha：遗忘因子

n = [na, nb, nc]; % 将三个阶次保存在数组中

switch flag
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(n,d);
  case 1
    sys=mdlDerivatives(t,x,u);
  case 2
    sys=mdlUpdate(t,x,u,n,d,alpha);
  case 3
    sys=mdlOutputs(t,x,u);
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(n,d)
global P K len %后面Update算法中要用到，定义为全局变量
len = sum(n) + 1; % 参数theta的长度

sizes = simsizes;
sizes.NumContStates  = 0;
% na多一个保存当前时刻的位置；nb多一个常数项以及保存d个时刻的位置；nc多一个保存当前时刻的位置
sizes.NumDiscStates  = len * 2 + d - 1; %前面len + d - 1为fai；后面len为theta
sizes.NumOutputs     = len; % theta
sizes.NumInputs      = 2; % 最外层输入信号和输出结果
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);

x0  = zeros(sizes.NumDiscStates,1);
str = [];
ts  = [-1 0];

P = eye(len) * 1e6;% 初始化P，矩阵阶次同fai
K = 0;
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
sys = [];

function sys=mdlUpdate(t,x,u,n,d,alpha)
na = n(1); nb = n(2); nc = n(3);
input = u(1); y = u(2);

global P K len

% 这里的x就是数据向量fai，也是初始化时的x0
y_part_rb = na; % y_part的右边界
y_part = x(1:y_part_rb); % na个
u_part_rb = y_part_rb + (nb+1) + d - 1; % u_part的右边界
u_part = x(y_part_rb+1 : u_part_rb); % nb + 1 + d - 1个
ksai_part_rb = u_part_rb + nc;
ksai_part = x(u_part_rb+1 : ksai_part_rb); % 共nc个
theta = x(ksai_part_rb + 1:end); %共len=na+nb+1+nc个  end-(len-1) = ksai_part_rb
% 构造数据向量
if nc > 0 
    fai = [y_part; u_part(d:end); ksai_part];
else
    fai = [y_part;u_part(d:end)];
end

% 计算当前时刻的K(k)，计算中用到的变量都是上一时刻的
K = (P * fai) / (alpha + fai' * P * fai);
% 计算当前时刻的参数估计值theta
theta = theta + K * (y - fai' * theta); % 这里计算的theta和后面的fai要一起更新到状态量当中
% 计算当前时刻的P
P = (eye(len) - K * fai') * P / alpha;
% 计算噪声估计值
ksai = y - fai' * theta;

% 更新状态量中输出y的部分
% t = 0.01时(k=1)，y(k-1)=y(0)进入fai(k-1),y(1)保存在y_part(0)
for i = length(y_part):-1:2
    y_part(i) = y_part(i-1);
end
y_part(1) = -y; % 记录当前时刻的-y
% 更新状态量中输入u的部分
for i = length(u_part):-1:2
    u_part(i) = u_part(i-1);
end
u_part(1) = input; % 记录当前时刻的u
fai_u = u_part(d:end);
% 更新状态量中噪声ksai的部分， 计算当前时刻的噪声ksai，
if nc > 0
    for i = length(ksai_part):-1:2
        ksai_part(i) = ksai_part(i-1);
    end
    ksai_part(1) = ksai; % 记录当前时刻的ksai,下一时刻用
end
% fai，theta更新到状态量中
x = [y_part;u_part;ksai_part;theta]; %[fai;theta]是错的，x是状态量，包括所有延时的量
sys = x;

function sys=mdlOutputs(t,x,u)
global len
theta = x(end-(len-1):end);
sys = theta;

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sys = [];

function sys=mdlTerminate(t,x,u)
sys = [];
