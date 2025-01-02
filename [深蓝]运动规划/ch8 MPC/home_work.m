clear all;
close all;
clc;

p_0 = [0 0 20];
v_0 = [0 0 0];
a_0 = [0 0 0];
K=20;
dt=0.2;

P=[];
P_ref = [];
V=[];
V_ref = [];
A=[];
A_ref = [];
w1 = 1000;
w2 = 10;
w3 = 10;
w4 = 1;

for t=0.2:0.2:80
    %% Construct the reference signal
    for i = 1:20
        tref = t + i*0.2;
        r=0.25*tref;
        pt(i,1) = r*sin(0.4*tref);
        vt(i,1) = r*cos(0.4*tref);
        at(i,1) = -r*sin(0.4*tref);
        
        pt(i,2) = r*cos(0.4*tref);
        vt(i,2) = -r*sin(0.4*tref);
        at(i,2) = -r*cos(0.4*tref);
        
        pt(i,3) = 20 - 0.4*tref;
        vt(i,3) = -0.5;
        at(i,3) = 0;
    end
    %% Do the MPC
    %% Construct the prediction matrix
    for k = 1:3
        [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K, dt, p_0(k), v_0(k), a_0(k));
        H = w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta);
        F = w1*(Bp - pt(:, k))'*Tp+w2*(Bv - pt(:, k))'*Tv+w3*(Ba - pt(:, k))'*Ta;
        J = quadprog(H,F,[],[]);
        j = J(1);
        p_0(k) = p_0(k) + v_0(k)*dt + 0.5*a_0(k)*dt^2 + 1/6*j*dt^3;
        v_0(k) = v_0(k) + a_0(k)*dt + 0.5*j*dt^2;
        a_0(k) = a_0(k) + j*dt; 
    end
   
    
    
    
    
    

    
    %% Log the states
    P_ref = [P_ref; pt(1,:)];
    P = [P;p_0];
    V_ref = [V_ref; vt(1,:)];
    V = [V;v_0];
    A_ref = [A_ref; at(1,:)];
    A = [A;a_0];
end

%% Plot the result
plot(P_ref, "r");
hold on;
plot(P, "b");
legend('参考轨迹位置','实际轨迹位置');
grid on;

figure;
plot(V_ref, "r");
hold on;
plot(V, "b");
legend('参考轨迹速度','实际轨迹速度');
grid on;

figure;
plot(A_ref, "r");
hold on;
plot(A, "b");
legend('参考轨迹加速度','实际轨迹加速度');
grid on;

figure;
plot3(P_ref(:,1),P_ref(:,2),P_ref(:,3), "r");
hold on;
plot3(P(:,1),P(:,2),P(:,3), "b");

axis equal;
