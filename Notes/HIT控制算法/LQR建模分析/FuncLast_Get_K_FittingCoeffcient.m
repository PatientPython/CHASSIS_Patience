% 创建者：林宸曳LuCkY，2025RM，平步电控
% 根据多个腿长求出不同的K矩阵，并对其进行关于腿长的拟合。

clear;
clc;
% 相关变量
Start = 0.08; % 开始拟合计算的腿长，单位为m
Step  = 0.01; % 每次增加的步长
End   = 0.38; % 结束拟合计算的腿长
Num = (End - Start)/Step + 1; % 左右腿分别的计算个数

% Q、R矩阵的赋值
% 位移、位移导、yaw、yaw导、左腿角度、左腿角度导、右腿角度、右腿角度导、pitch、pitch导
%s s_dot phi phi_dot theta_l theat_l_dot theta_r theta_r_dot pitch pitch_dot
% 左轮扭矩、右轮扭矩、左髋扭矩、右髋扭矩
% T_wl,T_wr,T_bl,T_br
Q = diag([650,250,300 80 6000 40 6000 40 200000 100]); 
R = diag([12 12 1.2 1.2]);

% 初始化40个零矩阵，这样子就可以稍微加快一点运算速度，虽然也没有太大必要
% （但是可以防止划黄线，这个黄线比较烦人，如果你不知道我在说什么你可以把这一段注释了看看）
k11  = zeros(Num);    k21 = zeros(Num);   k31 = zeros(Num);   k41 = zeros(Num);
k12  = zeros(Num);    k22 = zeros(Num);   k32 = zeros(Num);   k42 = zeros(Num);
k13  = zeros(Num);    k23 = zeros(Num);   k33 = zeros(Num);   k43 = zeros(Num);
k14  = zeros(Num);    k24 = zeros(Num);   k34 = zeros(Num);   k44 = zeros(Num);
k15  = zeros(Num);    k25 = zeros(Num);   k35 = zeros(Num);   k45 = zeros(Num);
k16  = zeros(Num);    k26 = zeros(Num);   k36 = zeros(Num);   k46 = zeros(Num);
k17  = zeros(Num);    k27 = zeros(Num);   k37 = zeros(Num);   k47 = zeros(Num);
k18  = zeros(Num);    k28 = zeros(Num);   k38 = zeros(Num);   k48 = zeros(Num);
k19  = zeros(Num);    k29 = zeros(Num);   k39 = zeros(Num);   k49 = zeros(Num);
k110 = zeros(Num);   k210 = zeros(Num);  k310 = zeros(Num);  k410 = zeros(Num);
% i和j用来表示矩阵的行和列
i = 1;
j = 1;

for l_l = Start : Step : End
    for l_r = Start : Step : End
        % 根据腿长和上面的QR矩阵算出K
        K_Temp = Func2_Get_K_Matrix(l_l,l_r,Q,R);

        % 将每次求出的值添加入矩阵中，方便后面的拟合
        k11(i,j) = K_Temp(1,1);  k21(i,j) = K_Temp(2,1);  k31(i,j) = K_Temp(3,1);  k41(i,j) = K_Temp(4,1);
        k12(i,j) = K_Temp(1,2);  k22(i,j) = K_Temp(2,2);  k32(i,j) = K_Temp(3,2);  k42(i,j) = K_Temp(4,2);
        k13(i,j) = K_Temp(1,3);  k23(i,j) = K_Temp(2,3);  k33(i,j) = K_Temp(3,3);  k43(i,j) = K_Temp(4,3);
        k14(i,j) = K_Temp(1,4);  k24(i,j) = K_Temp(2,4);  k34(i,j) = K_Temp(3,4);  k44(i,j) = K_Temp(4,4);
        k15(i,j) = K_Temp(1,5);  k25(i,j) = K_Temp(2,5);  k35(i,j) = K_Temp(3,5);  k45(i,j) = K_Temp(4,5);
        k16(i,j) = K_Temp(1,6);  k26(i,j) = K_Temp(2,6);  k36(i,j) = K_Temp(3,6);  k46(i,j) = K_Temp(4,6);
        k17(i,j) = K_Temp(1,7);  k27(i,j) = K_Temp(2,7);  k37(i,j) = K_Temp(3,7);  k47(i,j) = K_Temp(4,7);
        k18(i,j) = K_Temp(1,8);  k28(i,j) = K_Temp(2,8);  k38(i,j) = K_Temp(3,8);  k48(i,j) = K_Temp(4,8);
        k19(i,j) = K_Temp(1,9);  k29(i,j) = K_Temp(2,9);  k39(i,j) = K_Temp(3,9);  k49(i,j) = K_Temp(4,9);
        k110(i,j)= K_Temp(1,10); k210(i,j)= K_Temp(2,10); k310(i,j)= K_Temp(3,10); k410(i,j)= K_Temp(4,10);

        j=j+1;
    end
    j=1;
    i=i+1;
end

% 开始拟合，并且获得拟合后的多项式参数
% 每个计算后对应的参数是p00 p10 p01 p20 p11 p02 p30 p21 p12 p03
% 参数对应的变量如下，其中x是l_r，y是l_l
% p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 + p21*x^2*y + p12*x*y^2 + p03*y^3
l_l = Start : Step : End;
l_r = Start : Step : End;
% 把false改为true可以看到拟合曲面图（注意一次性不要画太多图，不然会很卡）
K11_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k11,true) );
K12_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k12,true) );
K13_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k13,true) );
K14_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k14,true) );
K15_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k15,true) );
K16_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k16,true) );
K17_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k17,true) );
K18_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k18,true) );
K19_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k19,true) );
K110_Fit = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k110,true) );

K21_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k21,true) );
K22_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k22,true) );
K23_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k23,true) );
K24_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k24,true) );
K25_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k25,true) );
K26_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k26,true) );
K27_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k27,true) );
K28_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k28,true) );
K29_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k29,true) );
K210_Fit = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k210,true) );

K31_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k31,true) );
K32_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k32,true) );
K33_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k33,true) );
K34_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k34,true) );
K35_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k35,true) );
K36_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k36,true) );
K37_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k37,true) );
K38_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k38,true) );
K39_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k39,true) );
K310_Fit = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k310,true) );

K41_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k41,true) );
K42_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k42,true) );
K43_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k43,true) );
K44_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k44,true) );
K45_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k45,true) );
K46_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k46,true) );
K47_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k47,true) );
K48_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k48,true) );
K49_Fit  = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k49,true) );
K410_Fit = coeffvalues( Func3_Fit_LegAndK(l_r,l_l,k410,true) );

% 打印所有拟合后的系数，方便抄到代码中
Func4_print_Coefficient(K11_Fit );
Func4_print_Coefficient(K12_Fit );
Func4_print_Coefficient(K13_Fit );
Func4_print_Coefficient(K14_Fit );
Func4_print_Coefficient(K15_Fit );
Func4_print_Coefficient(K16_Fit );
Func4_print_Coefficient(K17_Fit );
Func4_print_Coefficient(K18_Fit );
Func4_print_Coefficient(K19_Fit );
Func4_print_Coefficient(K110_Fit);
Func4_print_Coefficient(K21_Fit );
Func4_print_Coefficient(K22_Fit );
Func4_print_Coefficient(K23_Fit );
Func4_print_Coefficient(K24_Fit );
Func4_print_Coefficient(K25_Fit );
Func4_print_Coefficient(K26_Fit );
Func4_print_Coefficient(K27_Fit );
Func4_print_Coefficient(K28_Fit );
Func4_print_Coefficient(K29_Fit );
Func4_print_Coefficient(K210_Fit);
Func4_print_Coefficient(K31_Fit );
Func4_print_Coefficient(K32_Fit );
Func4_print_Coefficient(K33_Fit );
Func4_print_Coefficient(K34_Fit );
Func4_print_Coefficient(K35_Fit );
Func4_print_Coefficient(K36_Fit );
Func4_print_Coefficient(K37_Fit );
Func4_print_Coefficient(K38_Fit );
Func4_print_Coefficient(K39_Fit );
Func4_print_Coefficient(K310_Fit);
Func4_print_Coefficient(K41_Fit );
Func4_print_Coefficient(K42_Fit );
Func4_print_Coefficient(K43_Fit );
Func4_print_Coefficient(K44_Fit );
Func4_print_Coefficient(K45_Fit );
Func4_print_Coefficient(K46_Fit );
Func4_print_Coefficient(K47_Fit );
Func4_print_Coefficient(K48_Fit );
Func4_print_Coefficient(K49_Fit );
Func4_print_Coefficient(K410_Fit);


% % 下面这段代码可以用来测试拟合后的结果是否与直接计算出来的结果近似
% % 当然也可以直观地看上面代码的画图
% ll = 0.28;
% lr = 0.08;
% FitMatrix = [1; lr; ll; lr^2; lr*ll; ll^2; lr^3; lr^2*ll; lr*ll^2; ll^3];
% K = Func2_Get_K_Matrix(ll,lr,Q,R);
% % 拟合的结果
% K49_Fit
% K_Fitting = K49_Fit * FitMatrix 
% % 实际计算得出的结果
% K(4,9)

