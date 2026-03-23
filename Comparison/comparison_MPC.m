clc; clear all;

tic
%% -------------------- USER SETTINGS --------------------------------------
dt     = 0.1;
Tmax   = 90;
x0     = [0.35; 0.35; pi/6];     % [x;y;theta]
goal   = [2.65; 2.65];

% Obstacles: [cx cy radius]
obs = [0.75 0.75 0.25;
       1.75 1.75 0.25;
       0.25 1.75 0.25;
       2.25 0.75 0.25];

safety_buffer  = 0.05;                  
Reff = obs(:,3) + safety_buffer;

% Input limits
v_min = -0.25;   v_max = 0.8;          % allow small reverse
w_min = -2.5;    w_max = 2.5;

% Horizon
N   = 25;                              % prediction steps
Nx  = 3;   Nu = 2;
M   = size(obs,1);                     % # obstacles

% Cost weights
Qp  = 6.0 * eye(2);                    % position tracking per step
Qpsi= 0.5;                             % heading alignment (optional)
R   = diag([0.4, 0.05]);               % input effort
S   = diag([0.1, 0.02]);               % input rate (delta u)
Qf  = 30.0 * eye(2);                   % terminal position cost
Wslack = 2e4;                          % HUGE penalty per slack (obstacle softening)

% Goal stop thresholds
goal_pos_tol     = 0.01;
goal_heading_tol = 12*pi/180;

% fmincon options
opts = optimoptions('fmincon', 'Algorithm','sqp',...
    'Display','none', 'MaxIterations', 150, 'ConstraintTolerance',1e-4, ...
    'StepTolerance',1e-6, 'OptimalityTolerance',1e-4);

rng(0);


%% -------------------- Build optimization dimensions ----------------------
% Decision vector z = [u(0); ...; u(N-1); s(0); ...; s(N-1)]
% where u(k) = [v_k; w_k] (2 vars), and s(k) \in R^M are soft slacks for obstacles.
numU   = N*Nu;
numS   = N*M;
nz     = numU + numS;

% Bounds (box) for z
lb = [repmat([v_min; w_min], N, 1); zeros(numS,1)];     % slacks >= 0
ub = [repmat([v_max; w_max], N, 1); inf(numS,1)];

% Initial guess (warm start)
u_init = repmat([0.05; 0], N, 1);
s_init = 0.01 * ones(numS,1);
z      = [u_init; s_init];

%% -------------------- Sim loop -------------------------------------------
x = x0;  t = 0;
traj = x.';  ulog = [];  tlog = 0;

while t < Tmax
    % stop?
    e = goal - x(1:2);
    pos_err = norm(e);
    psi_goal = atan2(e(2), e(1));
    psi_err  = wrapToPi(psi_goal - x(3));
    if pos_err < goal_pos_tol && abs(psi_err) < goal_heading_tol
        disp('Goal reached successfully'); break;
    end

    % Nonlinear cost & constraints closures for current state
    costfun = @(z) stageCost(z, x, goal, obs, Reff, N, Qp, Qpsi, R, S, Qf, Wslack, dt, Nu, M);
    nonlcon = @(z) nlcon(z, x, obs, Reff, N, dt, Nu, M);  % only s>=0 handled via bounds

    % Solve MPC
    [z,~,exitflag] = fmincon(costfun, z, [],[],[],[], lb, ub, nonlcon, opts);
    if exitflag <= 0
        warning('fmincon failed (flag=%d). Applying first guess input.', exitflag);
    end

    % Extract first control
    u = z(1:Nu);
    v = u(1); w = u(2);

    % Integrate true unicycle
    x = rk4(@(xx,uu) f_unicycle(xx, uu), x, [v; w], dt);

    % Warm start: shift u & s
    z = shiftWarmStart(z, N, Nu, M);

    % Logs & plot
    t = t + dt;
    traj = [traj; x.'];
    ulog = [ulog; u.'];
    tlog = [tlog; t];
end

%% -------------------- Visualization --------------------------------------
figure(3);
hold on; grid on;
xlabel('x'); ylabel('y'); 
scatter(traj(1,1), traj(1,2), 36, 'm', 'filled');
scatter(goal(1), goal(2), 60, 'g', 'filled');
plot(traj(:,1), traj(:,2), 'b-', 'LineWidth', 2);
drawObstacles(obs(:,1), obs(:,2), obs(:,3), [0.95 0.6 0.6]);
legend('Start','Goal','Trajectory','Obstacles');
xlim([0 3])
ylim([0 3])

toc

%% ==================== COST FUNCTION ======================================
function J = stageCost(z, x0, goal, obs, Reff, N, Qp, Qpsi, R, S, Qf, Wslack, dt, Nu, M)
% Unpack
U = reshape(z(1:N*Nu), Nu, N);            % 2 x N
S_slack = reshape(z(N*Nu+1:end), M, N);   % M x N

% Rollout
x = x0;
J = 0;
u_prev = U(:,1);  % for rate at k=1 use difference from zero implicitly
for k = 1:N
    u = U(:,k);
    % cost: tracking
    e = goal - x(1:2);
    J = J + e.'*Qp*e + Qpsi*(wrapToPi(atan2(e(2),e(1)) - x(3)))^2 ...
          + u.'*R*u;
    % input rate
    if k>1
        du = u - U(:,k-1);
    else
        du = u - u_prev*0;
    end
    J = J + du.'*S*du;

    % slack penalty (sum over obstacles)
    s_k = S_slack(:,k);
    J = J + Wslack * (s_k.'*s_k);

    % step dynamics
    x = rk4(@(xx,uu) f_unicycle(xx,uu), x, u, dt);
end

% terminal cost
eN = goal - x(1:2);
J  = J + eN.'*Qf*eN;
end

%% ==================== NONLINEAR CONSTRAINTS ===============================
function [c, ceq] = nlcon(z, x0, obs, Reff, N, dt, Nu, M)
% Soft state constraints: for each k and obstacle i,
% g_i(k) = R_eff^2 - ||p_k - c_i||^2 + s_i(k) <= 0
% with s_i(k) >= 0 handled via bounds. This keeps feasibility.
U = reshape(z(1:N*Nu), Nu, N);
S_slack = reshape(z(N*Nu+1:end), M, N);

x = x0;
c = [];  % inequality stack
for k = 1:N
    u = U(:,k);
    x = rk4(@(xx,uu) f_unicycle(xx,uu), x, u, dt);
    pk = x(1:2);
    % obstacle constraints
    for i = 1:M
        d2 = sum((pk - obs(i,1:2).').^2);
        g  = Reff(i)^2 - d2 + S_slack(i,k);   % <= 0
        c  = [c; g]; %#ok<AGROW>
    end
end
ceq = [];  % no equality constraints
end

%% ==================== DYNAMICS & RK4 =====================================
function xdot = f_unicycle(x, u)
v = u(1); w = u(2);
xdot = [ v*cos(x(3));
         v*sin(x(3));
         w ];
end

function xnext = rk4(f, x, u, h)
k1 = f(x,        u);
k2 = f(x+0.5*h*k1, u);
k3 = f(x+0.5*h*k2, u);
k4 = f(x+    h*k3, u);
xnext = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

%% ==================== WARM START SHIFT ===================================
function z_next = shiftWarmStart(z, N, Nu, M)
% shift U forward one step, repeat last, same for slacks
U = reshape(z(1:N*Nu), Nu, N);
S = reshape(z(N*Nu+1:end), M, N);
U = [U(:,2:end), U(:,end)];
S = [S(:,2:end), S(:,end)];
z_next = [U(:); S(:)];
end

%% ==================== DRAW OBSTACLES =====================================
function drawObstacles(cx, cy, R, faceColor)
th = linspace(0,2*pi,120);
for i=1:numel(R)
    x = cx(i) + R(i)*cos(th);
    y = cy(i) + R(i)*sin(th);
    fill(x, y, faceColor, 'EdgeColor','r','FaceAlpha',0.35,'LineWidth',1.4);
end
end
