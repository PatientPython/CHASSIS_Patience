% 这份文件没用在拟合当中，仅仅是想单独求某个腿长的K的话
% 就用这个吧，赋值完毕后直接点运行即可，这样就可以单独算出某个腿长的K矩阵了

% 对左右腿长、Q、R赋值
% Q、R矩阵对应的状态变量如下
% 位移、位移导、yaw、yaw导、左腿角度、左腿角度导、右腿角度、右腿角度导、pitch、pitch导
%s s_dot phi phi_dot theta_l theat_l_dot theta_r theta_r_dot pitch pitch_dot
% 左轮扭矩、右轮扭矩、左髋扭矩、右髋扭矩
% T_wl,T_wr,T_bl,T_br
clear;
clc;
l_l = 0.14;
l_r = 0.14;
Q = diag([650,250,300 80 6000 40 6000 40 200000 100]); 
R = diag([12 12 1.2 1.2]);

% 获取AB矩阵，并计算K
[A,B] = Func1_Get_AB_Matrix(l_l,l_r);
K = lqr(A,B,Q,R);

% 显示出K的值
disp(K);