clc;clear;close all
v_max = 400;
a_max = 400;
color = ['r', 'b', 'm', 'g', 'k', 'c', 'c'];

%% specify the center points of the flight corridor and the region of corridor
% 看一下这里想要实现一下也是基于ginput()拿采样点的方法来做
% path = [50, 50;
%         100, 120;
%         180, 150;
%         250, 80;
%         280, 0];
path = [50, 50;
        100, 120;
        180, 150;
        250, 80];
x_length = 100;
y_length = 100;

n_order = 5;   % 8 control points
% FIXME: n_seg是不是有点问题啊? 这里注意!
n_seg = size(path, 1);

corridor = zeros(4, n_seg);

for i = 1:(n_seg)
    corridor(:, i) = [path(i, 1), path(i, 2), x_length/2, y_length/2]';
end

%% specify ts for each segment
ts = zeros(n_seg, 1);
for i = 1:n_seg
    ts(i,1) = 1;
end

poly_coef_x = MinimumSnapCorridorBezierSolver(1, path(:, 1), corridor, ts, n_seg, n_order, v_max, a_max);
poly_coef_y = MinimumSnapCorridorBezierSolver(2, path(:, 2), corridor, ts, n_seg, n_order, v_max, a_max);

%% display the trajectory and cooridor
plot(path(:,1), path(:,2), '*r'); hold on;
for i = 1:(n_seg)
    plot_rect([corridor(1,i);corridor(2,i)], corridor(3, i), corridor(4,i));hold on;
end
hold on;



%% #####################################################
% STEP 4: draw bezier curve
for k = 1:n_seg
    x_pos = [];y_pos = [];
    idx = 1;
    for t = 0:0.01:1
        x_pos(idx) = 0.0;
        y_pos(idx) = 0.0;
        for i = 0:n_order
            basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);
            x_pos(idx) = x_pos(idx) + poly_coef_x((k-1)*(n_order+1) + i+1) * basis_p;
            y_pos(idx) = y_pos(idx) + poly_coef_y((k-1)*(n_order+1) + i+1) * basis_p;
        end
        idx = idx + 1;
    end
  scatter(x_pos, y_pos, "filled", color(k));
  plot(x_pos, y_pos, color(k));
end


function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, n_seg, n_order, v_max, a_max)
    start_cond = [waypoints(1), 0, 0];
    end_cond   = [waypoints(end), 0, 0];   
    
    %% #####################################################
    % STEP 1: compute Q_0 of c'Q_0c
    [Q, M]  = getQM(n_seg, n_order, ts);
    Q_0 = M'*Q*M;
    Q_0 = nearestSPD(Q_0);
    
    %% #####################################################
    % STEP 2: get Aeq and beq
    [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond);
    
    %% #####################################################
    % STEP 3: get corridor_range, Aieq and bieq
    
    % STEP 3.1: get corridor_range of x-axis or y-axis,
    % you can define corridor_range as [p1_min, p1_max;
    %                                   p2_min, p2_max;
    %                                   ...,
    %                                   pn_min, pn_max ];
    corridor_range = zeros(n_seg, 2);
    for i = 1:n_seg
        corridor_range(i, 1) = corridor(axis, i) - corridor(axis + 2, i);
        corridor_range(i, 2) = corridor(axis, i) + corridor(axis + 2, i);
    end
    
    % STEP 3.2: get Aieq and bieq
    [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max);
    
    f = zeros(size(Q_0,1),1);
    poly_coef = quadprog(Q_0,f,Aieq, bieq, Aeq, beq);
end

function plot_rect(center, x_r, y_r)
    p1 = center+[-x_r;-y_r];
    p2 = center+[-x_r;y_r];
    p3 = center+[x_r;y_r];
    p4 = center+[x_r;-y_r];
    plot_line(p1,p2);
    plot_line(p2,p3);
    plot_line(p3,p4);
    plot_line(p4,p1);
end

function plot_line(p1,p2)
    a = [p1(:),p2(:)];    
    plot(a(1,:),a(2,:),'b');
end