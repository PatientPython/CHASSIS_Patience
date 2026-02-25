RM25-平步电控-林宸曳LuCkY

一、
	1、图片和文章用来参考，对照着看就可以知道里面的方程是怎么来的

	2、BalanceSoldier_LQR_ABMatrix是完整推导状态空间方程的，也就是AB矩阵怎么来的。

	3、BalanceSoldier_LQR_ABMatrix_ShangJiao是根据上交最后的5个方程直接求解，但最后解算没用这个。可以不看。

二、关于K的拟合计算
	4、Func1_Get_AB_Matrix由BalanceSoldier_LQR_ABMatrix生成，是用来获取AB矩阵的

	5、Func2_Get_K_Matrix是获取K矩阵的，需要传入的参数在文件中说明了

	6、Func3_Fit_LegAndK是拟合腿长和K矩阵用的，默认配置的是双相三次拟合（两个自变量，最高次数为3，对一个因变量拟合）。

	7、Func4_print_Coefficient打印用的，不用太细看。

	8、FuncLast_Get_K_FittingCoeffcient最终的代码文件，这份文件里面调用了前面的所有函数，计算出K的拟合系数。