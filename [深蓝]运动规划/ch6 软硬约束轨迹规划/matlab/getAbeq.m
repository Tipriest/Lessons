function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(3, n_all_poly);
    S = ts(1);
    k = 0; Aeq_start(k+1, 1:3) = [1,  0, 0]*S^(1-k);
    k = 1; Aeq_start(k+1, 1:3) = [-1, 1, 0]*n_order*S^(1-k);
    k = 2; Aeq_start(k+1, 1:3) = [1, -2, 1]*(n_order)*(n_order-1)*S^(1-k);
    beq_start = start_cond';
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(3, n_all_poly);
    S = ts(end);
    id_y = n_seg * (n_order+1) - 3;
    k = 0; Aeq_end(k+1, id_y+(1:3)) = [0,  0, 1]*S^(1-k);
    k = 1; Aeq_end(k+1, id_y+(1:3)) = [0, -1, 1]*n_order*S^(1-k);
    k = 2; Aeq_end(k+1, id_y+(1:3)) = [1, -2, 1]*n_order*(n_order-1)*S^(1-k);
    beq_end = end_cond';
    
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    k = 0;
    for i = 1:n_seg-1
        s1 = ts(i);
        s2 = ts(i+1);
        Aeq_con_p(i, i*(n_order+1)) = 1 * s1^(1-k);
        Aeq_con_p(i, i*(n_order+1)+1) = -1 * s2^(1-k);
    end

    beq_con_p = zeros(n_seg-1, 1);

    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    k = 1;
    for i = 1:n_seg-1
        s1 = ts(i);
        s2 = ts(i+1);
        Aeq_con_v(i, i*(n_order+1)-3 + (1:3)) = [0, -1, 1]*n_order*s1^(1-k);
        Aeq_con_v(i, i*(n_order+1)   + (1:3)) = (-1) * [-1, 1, 0]*n_order*s2^(1-k);
    end

    beq_con_v = zeros(n_seg-1, 1);

    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    k = 2;
    for i = 1:n_seg-1
        s1 = ts(i);
        s2 = ts(i+1);
        Aeq_con_a(i, i*(n_order+1)-3 + (1:3)) = [1, -2, 1]*n_order*(n_order-1)*s1^(1-k);
        Aeq_con_a(i, i*(n_order+1)   + (1:3)) = (-1) * [1, -2, 1]*n_order*(n_order-1)*s2^(1-k);
    end

    beq_con_a = zeros(n_seg-1, 1);

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end