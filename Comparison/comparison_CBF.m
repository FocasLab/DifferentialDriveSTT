clc; clear all;

tic
%% --------------------- USER SETTINGS -------------------------------------
dt     = 0.01;
Tmax   = 90;
x0     = [0.35; 0.35; pi/6];     % [x;y;theta]
goal   = [2.65; 2.65];

% Obstacles: [cx cy radius]
obs = [1 1 0.25;
       1.75 1.75 0.25;
       0.25 1.75 0.25;
       2.25 0.75 0.25];

safety_buffer = 0.05;
Reff = obs(:,3) + safety_buffer;

% Single-integrator (planar) limits
u_max = 0.9;                      % cap on ||u|| (speed in the plane)
u_box = u_max * [1; 1];           % simple box bound for QP

% Unicycle limits
v_min = 0;  v_max = 0.8;      % allow small reverse
w_min = -2.5;   w_max =  2.5;

% Nominal go-to-goal in the plane
k_p = 0.9;                         % proportional gain for u_nom

% CBF (for single-integrator): h_i = ||p - c_i||^2 - R_i^2
alpha = 2.0;                       % safe aggressiveness

% QP weights (min ||u - u_nom||^2 + w_slack * ||s||_2^2)
w_slack = 1e4;                     % big penalty → uses slack only if needed

% Goal stop
goal_pos_tol     = 0.001;
goal_heading_tol = 12*pi/180;

rng(0);

%% --------------------- Sim -----------------------------------------------
x = x0(:);  t = 0;
traj = x(:).'; ulog = []; tlog = 0;

M = size(obs,1);

while t < Tmax
    p = x(1:2);
    % ----- Stop at goal -----
    e  = goal - p;
    pos_err = norm(e);
    psi_goal = atan2(e(2), e(1));
    psi_err  = wrapToPi(psi_goal - x(3));
    if pos_err < goal_pos_tol && abs(psi_err) < goal_heading_tol
        disp('Goal reached safely.'); 
        break;
    end

    % ----- 1) Planar nominal control (single-integrator) -----
    u_nom = k_p * e;                       % go-to-goal in the plane
    if norm(u_nom) > u_max, u_nom = u_nom * (u_max/norm(u_nom)); end

    % QP: z = [u_x; u_y; s_1..s_M] with s_i >= 0
    H = blkdiag(2*eye(2), 2*w_slack*eye(M));
    f = [-2*u_nom; zeros(M,1)];

    % CBF_i: grad h_i^T u >= -alpha*h_i - s_i
    % grad h_i = 2(p - c_i)
    % -> [-grad h_i^T, 0..1_i..0] z <= alpha*h_i
    A = zeros(M, 2+M);
    b = zeros(M, 1);
    for i = 1:M
        di = p - obs(i,1:2).';
        h  = di.'*di - Reff(i)^2;
        gi = 2*di;                 % gradient wrt p
        A(i,1:2) = -gi.';          % -grad^T * u
        A(i,2+i) = 1;              % + s_i
        b(i)     = alpha*h;
    end

    % Bounds on u and slacks
    lb = [-u_box; zeros(M,1)];
    ub = [ u_box; inf(M,1)];

    options = optimoptions('quadprog','Display','off');
    [z,~,exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], options);

    if exitflag ~= 1
        % Very rare with slacks; still guard with tiny move toward goal
        warning('SI QP failed (flag=%d). Using damped u_nom.', exitflag);
        u = 0.2 * u_nom;
    else
        u = z(1:2);
    end

    % ----- 2) Near-identity mapping: u -> (v, w) for unicycle -----
    % Basis aligned with body:
    e_theta  = [cos(x(3)); sin(x(3))];
    e_thetaN = [-sin(x(3)); cos(x(3))];   % left-normal

    % Decompose desired planar velocity in body frame
    u_fwd = dot(u, e_theta);              % forward component along heading
    u_lat = dot(u, e_thetaN);             % lateral component (we turn to realize this)

    % Map:
    v = u_fwd;                            % drive forward as much as projection allows
    % Turn so that heading aligns with u: w ~ k_theta * atan2(u_lat, max(eps,|u_fwd|))
    k_theta = 4.0;
    w = k_theta * atan2(u_lat, max(1e-6, abs(u_fwd)));

    % Saturate unicycle inputs
    v = max(v_min, min(v_max, v));
    w = max(w_min, min(w_max, w));

    % ----- Integrate differential drive -----
    x = x + dt*[ v*cos(x(3)); v*sin(x(3)); w ];
    t = t + dt;

    traj = [traj; x(:).']; 
    ulog = [ulog; [v w]]; 
    tlog = [tlog; t];
end

%% --------------------- Viz -----------------------------------------------
figure(2);
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

%% -------------------- Helper: draw inflated obstacles ---------------------
function drawObstacles(cx, cy, R, faceColor)
th = linspace(0,2*pi,120);
for i=1:numel(R)
    x = cx(i) + R(i)*cos(th);
    y = cy(i) + R(i)*sin(th);
    fill(x, y, faceColor, 'EdgeColor','r','FaceAlpha',0.35,'LineWidth',1.4);
end
end
