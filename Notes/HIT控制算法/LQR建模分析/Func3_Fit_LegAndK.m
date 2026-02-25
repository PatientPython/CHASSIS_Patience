% K矩阵取值对腿长拟合的函数
% 传入参数：
%   左腿数据向量
%   右腿数据向量
%   Element：K矩阵的某个位置的元素构成的矩阵，与左右腿的参数对应
%   IsNeedFigure：是否需要绘图

% 传出参数：
%   fit函数返回的结果
function FitResult = Func3_Fit_LegAndK(l_r, l_l, Element, IsNeedFigure)

% 曲面拟合前，对参数进行预处理
[xData, yData, zData] = prepareSurfaceData(l_r, l_l, Element);

% 设置 fittype，并对数据进行模型拟合。
MyFitType = fittype('poly33');% 设置为双元素三次项拟合
FitResult = fit([xData, yData], zData, MyFitType);

% 根据传入的参数看是否需要绘图
if IsNeedFigure == true
    % 绘制数据拟合图
    figure( 'Name', '拟合腿长与K矩阵的某个元素' );
    h = plot( FitResult, [xData, yData], zData );
    legend( h, '拟合', 'Element 拟合 leg_left, leg_right', 'Location', 'NorthEast', 'Interpreter', 'none' );

    % 为坐标区加标签
    xlabel('Leg_right', 'Interpreter', 'none' );
    ylabel('Leg_left',  'Interpreter', 'none' );
    zlabel('Element',   'Interpreter', 'none' );
    grid on
end

end

% 创建者：林宸曳LuCkY，2025RM，平步电控