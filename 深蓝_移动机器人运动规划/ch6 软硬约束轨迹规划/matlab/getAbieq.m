function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint
    Aieq_p = zeros(2*n_seg*(n_order+1), n_all_poly);
    bieq_p = zeros(2*n_seg*(n_order+1), 1);
    k = 0;
    for i = 1 : n_seg
        for j = 1 : (n_order+1)
            s = ts(i);
            id_x = 2 * (n_order+1) * (i-1) + j;
            id_y = (n_order+1) * (i-1) + j;
            Aieq_p(id_x, id_y) = 1 * s;
            Aieq_p(id_x + (n_order + 1), id_y) = (-1) * 1 * s;
            bieq_p(id_x, 1) = corridor_range(i, 2);
            bieq_p(id_x + (n_order + 1), 1) = -corridor_range(i, 1);
        end
    end


    %#####################################################
    % STEP 3.2.2 v constraint
    Aieq_v = zeros(2*n_seg*(n_order+0), n_all_poly);
    bieq_v = zeros(2*n_seg*(n_order+0), 1);
    k = 1;
    for i = 1 : n_seg
        for j = 1 : (n_order+0)
            s = ts(i);
            id_x = 2 * (n_order+0) * (i-1) + j;
            id_y = (n_order+1) * (i-1) + j;
            Aieq_v(id_x, id_y+ (0:1)) = n_order * [-1, 1];
            Aieq_v(id_x + (n_order + 0), id_y+ (0:1)) = (-1) * n_order* [-1, 1];
            bieq_v(id_x, 1) = v_max;
            bieq_v(id_x + (n_order + 0), 1) = v_max;
        end
    end

    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = zeros(2*n_seg*(n_order-1), n_all_poly);
    bieq_a = zeros(2*n_seg*(n_order-1), 1);
    k = 2;
    for i = 1 : n_seg
        for j = 1 : (n_order-1)
            s = ts(i);
            id_x = 2 * (n_order-1) * (i-1) + j;
            id_y = (n_order+1) * (i-1) + j;
            Aieq_a(id_x, id_y+ (0:2)) = n_order*(n_order-1)/s* [1, -2, 1];
            Aieq_a(id_x + (n_order-1), id_y+ (0:2)) = (-1) * n_order*(n_order-1)/s* [1, -2, 1];
            bieq_a(id_x, 1) = a_max;
            bieq_a(id_x + (n_order-1), 1) = a_max;
        end
    end
    
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
end