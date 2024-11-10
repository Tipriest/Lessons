function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n = n_seg + 1; %n代表有n个点, n_seg代表有n_seg段
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    for k = 0 : 3                  % k代表起始点物理量的阶数，用0-3更合理
        for i = k : n_order        % i对应所有系数对应的权重
            x_id = k+1;
            y_id = i+1;
            Aeq_start(x_id, y_id) = factorial(i)/factorial(i-k)*0^(i-k);
        end
    end
    
    for i = 1 : 4
        beq_start(i, 1) = start_cond(i);
    end
    
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    for k = 0 : 3                 % k代表起始点物理量的阶数，用0-3更合理
        for i = k : n_order       % i对应所有系数对应的权重
            x_id = k+1;
            y_id = (n_order + 1)*(n-2) + i+1;
            Aeq_end(x_id, y_id) = factorial(i)/factorial(i-k)*ts(n_seg)^(i-k);
        end
    end
    
    
    for i = 1 : 4
        beq_end(i, 1) = end_cond(i);
    end
    
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    for j = 1 : n_seg-1       % 从第二段开始到最后一段第n段
        for k = 0 : 0           % k代表起始点物理量的阶数，用0-3更合理
            for i = k : n_order % i对应所有系数对应的权重
                x_id = j+k;
                y_id = (n_order + 1)*j + i + 1;
                Aeq_wp(x_id, y_id) = factorial(i)/factorial(i-k)*0^(i-k);
            end
        end
    end
    
    
    for i = 1 : n_seg-1
        beq_wp(i, 1) = waypoints(i+1);
    end
    
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    for j = 1 : n_seg - 1       % 对于中间的每一个点，中间有n_seg-2段，n_seg-1个点
        for k = 0 : 0           % k代表起始点物理量的阶数，用0-3更合理
            for i = k : n_order % i对应所有系数对应的权重
                Aeq_con_p(j+k, (n_order + 1)*(j-1) + i+1) = factorial(i)/factorial(i-k)*ts(j)^(i-k);
            end
            for i = k : n_order % i对应所有系数对应的权重
                Aeq_con_p(j+k, (n_order + 1)*j     + i+1) = -factorial(i)/factorial(i-k)*0^(i-k);
            end
        end
    end
    
    
    for i = 1 : n_seg-1
        beq_con_p(i, 1) = 0;
    end
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    for j = 1 : n_seg - 1       % 对于中间的每一个点，中间有n_seg-2段，n_seg-1个点
        for k = 1 : 1           % k代表起始点物理量的阶数，用0-3更合理
            for i = k : n_order % i对应所有系数对应的权重
                Aeq_con_v(j, (n_order + 1)*(j-1) + i+1) = factorial(i)/factorial(i-k)*ts(j)^(i-k);
            end
            for i = k : n_order % i对应所有系数对应的权重
                Aeq_con_v(j, (n_order + 1)*j     + i+1) = -factorial(i)/factorial(i-k)*0^(i-k);
            end
        end
    end
    
    
    for i = 1 : n_seg-1
        beq_con_v(i, 1) = 0;
    end

    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    for j = 1 : n_seg - 1       % 对于中间的每一个点，中间有n_seg-2段，n_seg-1个点
        for k = 2 : 2           % k代表起始点物理量的阶数，用0-3更合理
            for i = k : n_order % i对应所有系数对应的权重
                Aeq_con_a(j, (n_order + 1)*(j-1) + i+1) = factorial(i)/factorial(i-k)*ts(j)^(i-k);
            end
            for i = k : n_order % i对应所有系数对应的权重
                Aeq_con_a(j, (n_order + 1)*j     + i+1) = -factorial(i)/factorial(i-k)*0^(i-k);
            end
        end
    end
    
    
    for i = 1 : n_seg-1
        beq_con_a(i, 1) = 0;
    end
    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    for j = 1 : n_seg - 1       % 对于中间的每一个点，中间有n_seg-2段，n_seg-1个点
        for k = 3 : 3           % k代表起始点物理量的阶数，用0-3更合理
            for i = k : n_order % i对应所有系数对应的权重
                Aeq_con_j(j, (n_order + 1)*(j-1) + i+1) = factorial(i)/factorial(i-k)*ts(j)^(i-k);
            end
            for i = k : n_order % i对应所有系数对应的权重
                Aeq_con_j(j, (n_order + 1)*j     + i+1) = -factorial(i)/factorial(i-k)*0^(i-k);
            end
        end
    end
    
    
    for i = 1 : n_seg-1
        beq_con_j(i, 1) = 0;
    end
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end