function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        M_k = [];
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
    for l = 0 : (n_order-1)/2          % l代表起始点物理量的阶数，用0-3更合理
        for i = l : n_order            % i对应所有系数对应的权重
            x_id = l+1;
            y_id = i+1;
            M_k(x_id, y_id)               = factorial(i)/factorial(i-l)*0^(i-l);
            M_k(x_id+(n_order-1)/2+1, y_id) = factorial(i)/factorial(i-l)*ts(k)^(i-l);
        end
    end

        M = blkdiag(M, M_k);
    end
end