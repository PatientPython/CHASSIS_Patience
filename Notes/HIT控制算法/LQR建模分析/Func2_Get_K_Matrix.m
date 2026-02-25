% 获取K矩阵的函数
% 传入参数：左腿长、右腿长、Q矩阵、R矩阵
% 传出参数：K矩阵

function K = Func2_Get_K_Matrix(l_l,l_r,Q,R)

    [A,B] = Func1_Get_AB_Matrix(l_l,l_r);
    K = lqr(A,B,Q,R);
end

% 创建者：林宸曳LuCkY，2025RM，平步电控