function [Q, M] = getQM(n_seg, n_order, ts)
    Q = [];
    M = [];
    M_k = getM(n_order);
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 calculate Q_k of the k-th segment 
        Q_k = zeros(n_order+1, n_order+1);
        t_k = 1;
        s_k = ts(k);
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment
        for i = 4:n_order
            for l = 4:n_order
                Q_k(i+1, l+1) = (i * (i-1) * (i-2) * (i-3) * l * (l-1) * (l-2) * (l-3) / (i+l-7)) * t_k^(i+l-7) / s_k ^(2*4-3);
                % Q_k(i+1, l+1) = factorial(i)/factorial(i-4)*factorial(l)/factorial(l-4)/(i+l-7)*ts(k)^(i+l+7);
            end
        end

        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end