function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = zeros(n_order+1, n_order+1);
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment
        for i = 4:n_order
            for l = 4:n_order
                Q_k(i+1, l+1) = (i * (i-1) * (i-2) * (i-3) * l * (l-1) * (l-2) * (l-3) / (i+l-7)) * (ts(k)^(i+l-7)) ;
                % Q_k(i+1, l+1) = factorial(i)/factorial(i-4)*factorial(l)/factorial(l-4)/(i+l-7)*ts(k)^(i+l+7);
            end
        end
        Q = blkdiag(Q, Q_k);
    end
end