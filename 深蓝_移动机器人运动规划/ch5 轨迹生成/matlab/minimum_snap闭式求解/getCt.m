function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    Ct = zeros(n_seg * (n_order + 1), (n_order+1) + (n_seg-1)*(n_order+1)/2);
    n_constrain = (n_order + 1) + (n_seg-1);
    %包含第一段  0时刻的(n_order + 1)/2个参数
    %包含最后一段T时刻的(n_order + 1)/2个参数
    %包含中间的(n_seg-1)个点的位置
    n_free = (n_order+1)/2*(n_seg+1) - n_constrain;
    for i = 1 : (n_order+1)/2
        Ct(i, i) = 1;
    end
    for i = 1 : (n_order+1)/2
        id_x = n_seg * (n_order+1) - (n_order+1)/2 + i;
        id_y = (n_order+1)/2 + (n_seg-1) + i;
        Ct(id_x, id_y) = 1;
    end
    % 对于中间的n_seg-1个点的处理
    for i = 1 : (n_seg-1)
        for j = 1 : (n_order+1)/2 %重复的迭代处理
            id_x = (n_order+1)/2 + (n_order+1)*(i-1)+j;
            if j == 1
                id_y = (n_order+1)/2 +i;
            else
                id_y = n_constrain + ((n_order+1)/2-1)*(i-1)+ (j-1);
            end
            Ct(id_x, id_y) = 1;
            Ct(id_x + (n_order+1)/2, id_y) = 1;
        end
    end
end