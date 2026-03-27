clc; clear all; close all;

%% --------------------- GLOBAL SETTINGS -----------------------------------
dt_sim = 0.01;         % Common simulation time step
Tmax   = 60;
x0     = [0.35; 0.35; pi/6]; 
goal   = [2.65; 2.65];
obs    = [1 1 0.25; 1.75 1.75 0.25; 0.25 1.75 0.25; 2.25 0.75 0.25];
M      = size(obs,1);
goal_pos_tol = 0.05;

% Pre-allocate results table
results = table('Size',[3 3], 'VariableTypes',{'string','double','double'}, ...
    'VariableNames',{'Method','Total_Time(s)','Avg_Time_Per_Step (ms)'});

N_runs = 100; % number of Monte Carlo runs
use_disturbance = [1]; % 0: no disturbance, 1: with disturbance


for d_case = 1:length(use_disturbance)
    disturb_flag = use_disturbance(d_case);
    fprintf('\n===== Disturbance = %d =====\n', disturb_flag);

    %% --------------------- CASE 1: CBF (Single-Integrator QP) ----------------
    % fprintf('Running CBF Simulation...\n');
    % success_count = 0; time_array = zeros(N_runs,1);
    % for run = 1:N_runs
    %     x = x0; t = 0; step_count = 0;
    %     alpha = 2.0; w_slack = 1e4; Reff = obs(:,3) + 0.05;
    %     u_max = 0.9; u_box = u_max * [1; 1]; k_p = 0.9;
    %     v_min = 0; v_max = 0.8; w_min = -2.5; w_max =  2.5;
    %     traj = x(:).'; ulog = []; tlog = 0;
    % 
    %     tic;
    %     while t < Tmax
    %         p = x(1:2);
    %         % ----- Stop at goal -----
    %         e  = goal - p;
    %         pos_err = norm(e);
    % 
    %         % ----- 1) Planar nominal control (single-integrator) -----
    %         u_nom = k_p * e;                       % go-to-goal in the plane
    %         if norm(u_nom) > u_max, u_nom = u_nom * (u_max/norm(u_nom)); end
    % 
    %         % QP: z = [u_x; u_y; s_1..s_M] with s_i >= 0
    %         H = blkdiag(2*eye(2), 2*w_slack*eye(M));
    %         f = [-2*u_nom; zeros(M,1)];
    % 
    %         % CBF_i: grad h_i^T u >= -alpha*h_i - s_i
    %         % grad h_i = 2(p - c_i)
    %         % -> [-grad h_i^T, 0..1_i..0] z <= alpha*h_i
    %         A = zeros(M, 2+M);
    %         b = zeros(M, 1);
    %         for i = 1:M
    %             di = p - obs(i,1:2).';
    %             h  = di.'*di - Reff(i)^2;
    %             gi = 2*di;                 % gradient wrt p
    %             A(i,1:2) = -gi.';          % -grad^T * u
    %             A(i,2+i) = 1;              % + s_i
    %             b(i)     = alpha*h;
    %         end
    % 
    %         % Bounds on u and slacks
    %         lb = [-u_box; zeros(M,1)];
    %         ub = [ u_box; inf(M,1)];
    % 
    %         options = optimoptions('quadprog','Display','off');
    %         % [z,~,exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], options);
    %         [z,~,exitflag] = quadprog(H, f, A, b, [], [], [], [], [], options);
    % 
    %         if exitflag ~= 1
    %             % Very rare with slacks; still guard with tiny move toward goal
    %             warning('SI QP failed (flag=%d). Using damped u_nom.', exitflag);
    %             u = 0.2 * u_nom;
    %         else
    %             u = z(1:2);
    %         end
    % 
    %         % ----- 2) Near-identity mapping: u -> (v, w) for unicycle -----
    %         % Basis aligned with body:
    %         e_theta  = [cos(x(3)); sin(x(3))];
    %         e_thetaN = [-sin(x(3)); cos(x(3))];   % left-normal
    % 
    %         % Decompose desired planar velocity in body frame
    %         u_fwd = dot(u, e_theta);              % forward component along heading
    %         u_lat = dot(u, e_thetaN);             % lateral component (we turn to realize this)
    % 
    %         % Map:
    %         v = u_fwd;                            % drive forward as much as projection allows
    %         % Turn so that heading aligns with u: w ~ k_theta * atan2(u_lat, max(eps,|u_fwd|))
    %         k_theta = 4.0;
    %         w = k_theta * atan2(u_lat, max(1e-6, abs(u_fwd)));
    % 
    %         % Saturate unicycle inputs
    %         v = max(v_min, min(v_max, v));
    %         w = max(w_min, min(w_max, w));
    % 
    %         % add disturbance
    %         if disturb_flag == 1
    %             db = 1e-4 + (1.25e-2 - 1e-4) * (run - 1)/(N_runs - 1);
    %             dist = db*[1;1;1]*sin(t);
    %         else
    %             dist = zeros(3,1);
    %         end
    % 
    %         % ----- Integrate differential drive -----
    %         x = x + dt_sim*([v*cos(x(3)); v*sin(x(3)); w ]+dist);
    %         t = t + dt_sim;
    % 
    %         traj = [traj; x(:).']; 
    %         ulog = [ulog; [v w]]; 
    %         tlog = [tlog; t];p = x(1:2);
    %         step_count = step_count + 1;
    %     end
    %     elapsed = toc;
    %     time_array(run) = (elapsed/step_count)*1000;
    %     safe = true;
    %     for j = 1:M
    %         if any(vecnorm(traj(:,1:2)-obs(j,1:2),2,2) <= Reff(j))
    %             safe = false;
    %             break;
    %         end
    %     end
    %     if norm(goal - x(1:2)) <= 0.5 && safe
    %         success_count = success_count + 1;
    %     end
    % end
    % fprintf('CBF Time: %.4f ± %.4f ms | Success Rate: %.2f%%', mean(time_array), std(time_array), 100*success_count/N_runs);
    % %% --------------------- CASE 2: STT (Differential Drive) ------------------
    % fprintf('\n Running STT Simulation...\n');
    % success_count = 0; time_array = zeros(N_runs,1);
    % for run = 1:N_runs
    %     format long
    %     q = readmatrix("comparison.xlsx");
    %     start = [0.35 0.35 0.2];
    %     target = [2.65 2.65 0.2];
    %     obs = [0.75 0.75 0.25;
    %            1.75 1.75 0.25;
    %            0.25 1.75 0.25;
    %            2.25 0.75 0.25];
    %     Reff = obs(:,3) + 0.05;
    % 
    %     dt = 1e-4;
    %     t = (0:dt:90)';
    %     step_count = 0;
    %     global edbar
    %     edbar = 1e-1;
    % 
    %     kd = 0.1;
    %     ktheta = 10.1;
    %     % Initialization
    %     N = length(t);
    %     ed = zeros(1,N);
    %     etheta = zeros(1,N);
    %     eps_d = zeros(1,N);
    %     eps_theta = zeros(1,N);
    %     alpha_d = zeros(1,N);
    %     alpha_theta = zeros(1,N);
    %     delta = zeros(1,N);
    %     ux = zeros(1,N);
    %     uy = zeros(1,N);
    %     xd = zeros(1,N+1);
    %     yd = zeros(1,N+1);
    %     thetad = zeros(1,N+1);
    %     vd = zeros(1,N);
    %     wd = zeros(1,N);
    % 
    %     cenx = [ones(length(t),1), t, t.^2, t.^3]*q(2:5);
    %     ceny = [ones(length(t),1), t, t.^2, t.^3]*q(6:9);
    % 
    %     rad = q(10);
    %     thetad(1) = 0;
    %     xd(1) = 0.32;
    %     yd(1) = 0.35;
    %     t = t';
    %     cenx = cenx';
    %     ceny = ceny';
    %     rhod_zero = 0.45;
    %     rhod_inf = 0.01;
    %     rhotheta_zero = 1.5;
    %     rhotheta_inf = 0.1;
    %     decay_d = 0.2/100;
    %     decay_theta = 0.05/10;
    %     rhod = (rhod_zero - rhod_inf)*exp(-decay_d*t) + rhod_inf;
    %     rhotheta = (rhotheta_zero - rhotheta_inf)*exp(-decay_theta*t) + rhotheta_inf;
    % 
    %     tic
    %     dm = -0.01;
    %     for i = 1:N-1
    %         ed(i) = norm([xd(i)-cenx(i), yd(i)-ceny(i)])/rad;
    %         etheta(i) = psi(ed(i))*(2/pi)*(atan2(ceny(i)-yd(i),cenx(i)-xd(i))-thetad(i));
    % 
    %         ed(i) = min(max(ed(i) / rhod(i),-1+dm),1-dm);
    %         etheta(i) = min(max(etheta(i) / rhotheta(i),-1+dm),1-dm);
    % 
    %         eps_d(i) = log((1+ed(i))/(1-ed(i)));
    %         eps_theta(i) = log((1+etheta(i))/(1-etheta(i)));
    % 
    %         alpha_d(i) = 2/((1-ed(i)^2)*rhod(i)*rad);
    %         alpha_theta(i) = 4/((1-etheta(i)^2)*pi*rhotheta(i)*ed(i)*rad);
    %         delta(i) = atan2(ceny(i)-yd(i),cenx(i)-xd(i));
    % 
    %         vd(i) = kd*(eps_d(i)*alpha_d(i)*cos(delta(i)-thetad(i)) - eps_theta(i)*alpha_theta(i)*sin(delta(i)-thetad(i)));
    %         wd(i) = ktheta*eps_theta(i)*alpha_theta(i);
    % 
    %         if disturb_flag == 1
    %             db = 1e-4 + (1.25e-2 - 1e-4) * (run - 1)/(N_runs - 1);
    %             dist = db*[1;1;1]*sin(t(i));
    %         else
    %             dist = zeros(3,1);
    %         end
    % 
    %         xd(i+1) = xd(i) + dt*(vd(i)*cos(thetad(i)) + dist(1));
    %         yd(i+1) = yd(i) + dt*(vd(i)*sin(thetad(i)) + dist(2));
    %         thetad(i+1) = thetad(i) + dt*(wd(i) + dist(3));
    % 
    %         step_count= step_count + 1; 
    %     end
    % 
    %     elapsed = toc;
    %     time_array(run) = (elapsed/step_count)*1000;
    %     safe = true;
    %     for j = 1:M
    %         if any(vecnorm([xd', yd']-obs(j,1:2),2,2) <= Reff(j))
    %             safe = false;
    %             break;
    %         end
    %     end
    %     if norm(goal - [xd(end-1);yd(end-1)]) <= 0.5 && safe
    %         success_count = success_count + 1;
    %     end
    % end
    % fprintf('STT Time: %.4f ± %.4f ms | Success Rate: %.2f%%', mean(time_array), std(time_array), 100*success_count/N_runs);
    
    %% --------------------- CASE 3: MPC (Nonlinear fmincon) -------------------
    fprintf('\n Running MPC Simulation...\n');
    success_count = 0; time_array = zeros(N_runs,1);
    for run = 1%:(N_runs/2)
        run
        safety_buffer  = 0.05; x0 = [0.35; 0.35; pi/6]; dt = dt_sim*10;                
        Reff = obs(:,3) + safety_buffer; step_count = 0;
        
        % Input limits
        v_min = -0.25;   v_max = 0.8;          % allow small reverse
        w_min = -2.5;    w_max = 2.5;
        
        % N   = 25;                              % prediction steps
        N   = 15;                              % prediction steps
        Nx  = 3;   Nu = 2;
        M   = size(obs,1);                     % # obstacles
        
        Qp  = 6.0 * eye(2);                    % position tracking per step
        Qpsi= 0.5;                             % heading alignment (optional)
        R   = diag([0.4, 0.05]);               % input effort
        S   = diag([0.1, 0.02]);               % input rate (delta u)
        Qf  = 30.0 * eye(2);                   % terminal position cost
        % Wslack = 2e4;                          % HUGE penalty per slack (obstacle softening)
        Wslack = 1e3;                          % HUGE penalty per slack (obstacle softening)
        
        goal_pos_tol     = 0.25;
        goal_heading_tol = 12*pi/180;
        
        % opts = optimoptions('fmincon', 'Algorithm','sqp',...
        %     'Display','none', 'MaxIterations', 150, 'ConstraintTolerance',1e-4, ...
        %     'StepTolerance',1e-6, 'OptimalityTolerance',1e-4);
        opts = optimoptions('fmincon', ...
                            'Algorithm','sqp', ...
                            'Display','none', ...
                            'MaxIterations', 300, ...
                            'ConstraintTolerance',1e-3, ...
                            'StepTolerance',1e-5, ...
                            'OptimalityTolerance',1e-3);
        
        rng(0);
        
        numU   = N*Nu;
        numS   = N*M;
        nz     = numU + numS;
        
        lb = [repmat([v_min; w_min], N, 1); zeros(numS,1)];     % slacks >= 0
        ub = [repmat([v_max; w_max], N, 1); inf(numS,1)];
        
        % u_init = repmat([0.05; 0], N, 1);
        u_init = repmat([0.2; 0], N, 1);
        s_init = 0.01 * ones(numS,1);
        z      = [u_init; s_init];
        
        x = x0;  t = 0;
        traj = x.';  ulog = [];  tlog = 0;
        
        tic
        while t < Tmax
            % stop?
            e = goal - x(1:2);
            pos_err = norm(e);
            psi_goal = atan2(e(2), e(1));
            psi_err  = wrapToPi(psi_goal - x(3));
            if pos_err < goal_pos_tol && abs(psi_err) < goal_heading_tol
                break;
            end
        
            % Nonlinear cost & constraints closures for current state
            costfun = @(z) stageCost(z, x, goal, obs, Reff, N, Qp, Qpsi, R, S, Qf, Wslack, dt, Nu, M);
            nonlcon = @(z) nlcon(z, x, obs, Reff, N, dt, Nu, M); 
        
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
            
            if disturb_flag == 1
                db = 1e-4 + (1.25e-2 - 1e-4) * (run - 1)/((N_runs/5) - 1);
                dist = db*[1;1;1]*sin(t);
            else
                dist = zeros(3,1);
            end

            x = x + dt * dist;
        
            % Warm start: shift u & s
            z = shiftWarmStart(z, N, Nu, M);
        
            % Logs & plot
            t = t + dt;
            traj = [traj; x.'];
            ulog = [ulog; u.'];
            tlog = [tlog; t];
            step_count = step_count + 1;
        end

        elapsed = toc;
        time_array(run) = (elapsed/step_count)*1000;
        safe = true;
        for j = 1:M
            if any(vecnorm(traj(:,1:2)-obs(j,1:2),2,2) < Reff(j) - 0.05)
                min(vecnorm(traj(:,1:2)-obs(j,1:2),2,2) - Reff(j))
                safe = false;
                break;
            end
        end
        if norm(goal - x(1:2)) <= 0.5 && safe
            success_count = success_count + 1;
        end
        success_count
    end
    
    time_array = time_array(1:N_runs/5);
    fprintf('MPC Time: %.4f ± %.4f ms | Success Rate: %.2f%%', mean(time_array), std(time_array), 100*success_count*5/N_runs);
end

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

%% --------------------- HELPER FUNCTIONS ----------------------------------
function J = stageCost(z, x0, goal, obs, Reff, N, Qp, Qpsi, R, S, Qf, Wslack, dt, Nu, M, dist)
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

function [c, ceq] = nlcon(z, x0, obs, Reff, N, dt, Nu, M)
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
            c  = [c; g]; 
        end
    end
    ceq = [];  % no equality constraints
end


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

function z_next = shiftWarmStart(z, N, Nu, M)
% shift U forward one step, repeat last, same for slacks
    U = reshape(z(1:N*Nu), Nu, N);
    S = reshape(z(N*Nu+1:end), M, N);
    U = [U(:,2:end), U(:,end)];
    S = [S(:,2:end), S(:,end)];
    z_next = [U(:); S(:)];
end

function act = psi(x)
        global edbar
        Delta = 0.01;
        if x < Delta*edbar
            act = 0; % Default action
        elseif x >= Delta * edbar && x < 1
            act =  (x-Delta* edbar) / (1-Delta*edbar); % Action when x is greater than or equal to 0.99
        else 
            act = 1;
        end
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
