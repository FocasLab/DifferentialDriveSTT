clc; clear all; close all;

%% --------------------- GLOBAL SETTINGS -----------------------------------
dt_sim = 0.01;         % Common simulation time step
Tmax   = 60;
x0     = [0.35; 0.35; pi/6]; 
goal   = [2.65; 2.65];
obs    = [0.75 0.75 0.25; 1.75 1.75 0.25; 0.25 1.75 0.25; 2.25 0.75 0.25];
M      = size(obs,1);
goal_pos_tol = 0.05;

% Pre-allocate results table
results = table('Size',[3 3], 'VariableTypes',{'string','double','double'}, ...
    'VariableNames',{'Method','Total_Time(s)','Avg_Time_Per_Step (ms)'});

%% --------------------- CASE 1: CBF (Single-Integrator QP) ----------------
fprintf('Running CBF Simulation...\n');
x = x0; t = 0; step_count = 0;
alpha = 2.0; w_slack = 1e4; Reff = obs(:,3) + 0.05;
u_max = 0.9;

tic;
while t < Tmax && norm(goal - x(1:2)) > goal_pos_tol
    p = x(1:2);
    u_nom = 0.9 * (goal - p);
    if norm(u_nom) > u_max, u_nom = u_nom * (u_max/norm(u_nom)); end
    
    % QP Setup
    H = blkdiag(2*eye(2), 2*w_slack*eye(M));
    f = [-2*u_nom; zeros(M,1)];
    A = zeros(M, 2+M); b = zeros(M, 1);
    for i = 1:M
        di = p - obs(i,1:2).';
        h  = di.'*di - Reff(i)^2;
        A(i,1:2) = -(2*di).'; A(i,2+i) = 1; b(i) = alpha*h;
    end
    
    [z,~,exitflag] = quadprog(H, f, A, b, [], [], [-u_max;-u_max;zeros(M,1)], [u_max;u_max;inf(M,1)], [], optimoptions('quadprog','Display','off'));
    u = z(1:2);
    
    % Unicycle Mapping
    v = dot(u, [cos(x(3)); sin(x(3))]);
    w = 4.0 * atan2(dot(u, [-sin(x(3)); cos(x(3))]), max(1e-6, abs(v)));
    
    x = x + dt_sim * [v*cos(x(3)); v*sin(x(3)); w];
    t = t + dt_sim; step_count = step_count + 1;
end
cbf_total = toc;
results(1,:) = {"CBF-QP", cbf_total, (cbf_total/step_count)*1000};

%% --------------------- CASE 2: STT (Differential Drive) ------------------
fprintf('Running STT Simulation...\n');
format long
q = readmatrix("comparison.xlsx");
start = [0.35 0.35 0.2];
target = [2.65 2.65 0.2];
obs = [0.75 0.75 0.25;
       1.75 1.75 0.25;
       0.25 1.75 0.25;
       2.25 0.75 0.25];

dt = 1e-4;
t = (0:dt:90)';
% Initialization
N = length(t);
ux = zeros(1,N);
uy = zeros(1,N);
xd = zeros(1,N);
yd = zeros(1,N);
thetad = zeros(1,N);
vd = zeros(1,N);
wd = zeros(1,N);
cenx = [ones(length(t),1), t, t.^2, t.^3]*q(2:5);
ceny = [ones(length(t),1), t, t.^2, t.^3]*q(6:9);
rad = q(10);
thetad(1) = atan2(ceny(2)-ceny(1), cenx(2)-cenx(1));
xd(1) = 0.34;
yd(1) = 0.35;
t = t';
cenx = cenx';
ceny = ceny';

for i=2:N
    % STT
    ux(i) = -10*(xd(i-1)-cenx(i));
    uy(i) = -10*(yd(i-1)-ceny(i));
    
    % Differential
    vd(i) = sqrt(ux(i)^2 + uy(i)^2);
    wd(i) = psi((ux(i)^2 + uy(i)^2)/1e-6) * ( ux(i)*( (uy(i)-uy(i-1))/dt ) - uy(i)*( (ux(i)-ux(i-1))/dt ) ) / ( ux(i)^2 + uy(i)^2 );
    xd(i) = xd(i-1) + dt*vd(i)*cos(thetad(i-1));
    yd(i) = yd(i-1) + dt*vd(i)*sin(thetad(i-1));
    thetad(i) = thetad(i-1) + dt*wd(i);
end

stt_total = toc;
results(2,:) = {"STT (Diff-Drive)", stt_total, (stt_total/step_count)*1000};

%% --------------------- CASE 3: MPC (Nonlinear fmincon) -------------------
fprintf('Running MPC Simulation...\n');
x = x0; t = 0; step_count = 0;
N_hor = 10; % Reduced horizon for speed comparison
opts = optimoptions('fmincon','Display','none','Algorithm','sqp');
z_ws = zeros(N_hor*2 + N_hor*M, 1); % Warm start

tic;
while t < Tmax && norm(goal - x(1:2)) > goal_pos_tol
    % Solve one step of MPC
    cost = @(z) mpc_cost(z, x, goal, N_hor, dt_sim, M);
    con  = @(z) mpc_con(z, x, obs, Reff, N_hor, dt_sim, M);
    
    [z_sol,~] = fmincon(cost, z_ws, [],[],[],[], -inf(size(z_ws)), inf(size(z_ws)), con, opts);
    u = z_sol(1:2);
    
    x = x + dt_sim * [u(1)*cos(x(3)); u(1)*sin(x(3)); u(2)];
    t = t + dt_sim; step_count = step_count + 1;
    z_ws = z_sol; % Basic warm start
end
mpc_total = toc;
results(3,:) = {"MPC (fmincon)", mpc_total, (mpc_total/step_count)*1000};

%% --------------------- FINAL DISPLAY -------------------------------------
fprintf('\n--- Performance Comparison ---\n');
disp(results);

%% --------------------- HELPER FUNCTIONS ----------------------------------
function J = mpc_cost(z, x, goal, N, dt, M)
    U = reshape(z(1:N*2), 2, N);
    J = 0; curr_x = x;
    for i=1:N
        curr_x = curr_x + dt*[U(1,i)*cos(curr_x(3)); U(1,i)*sin(curr_x(3)); U(2,i)];
        J = J + norm(curr_x(1:2)-goal)^2 + 0.1*norm(U(:,i))^2;
    end
end

function [c, ceq] = mpc_con(z, x, obs, Reff, N, dt, M)
    U = reshape(z(1:N*2), 2, N);
    c = []; curr_x = x;
    for i=1:N
        curr_x = curr_x + dt*[U(1,i)*cos(curr_x(3)); U(1,i)*sin(curr_x(3)); U(2,i)];
        for j=1:M
            dist_sq = sum((curr_x(1:2) - obs(j,1:2)').^2);
            c = [c; Reff(j)^2 - dist_sq]; 
        end
    end
    ceq = [];
end