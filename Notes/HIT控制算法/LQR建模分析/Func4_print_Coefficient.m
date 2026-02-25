function Func4_print_Coefficient(K_Fit)

    fprintf('{%.4f',K_Fit(1,1));
    for i = 2:10
        fprintf(", %.4f",K_Fit(1,i));
    end
    fprintf("}\n");
end