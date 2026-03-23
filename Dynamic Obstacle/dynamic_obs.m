clc
clear all

format long
q = readmatrix("dynamic_tube.xlsx");
M = size(q,1);
q1 = q(1,:);
q2 = q(2,:);
q3 = q(3,:);
q4 = q(4,:);

dt = 1e-4;
t0 = 0;
tf = 5;
t = (t0:dt:tf)';
N = length(t);

breaks = linspace(t0, tf, M+1);

%% Main robot trajectory
ux_R = zeros(1,N);
uy_R = zeros(1,N);
xd_R = zeros(1,N);
yd_R = zeros(1,N);
thetad_R = zeros(1,N);
vd_R = zeros(1,N);
wd_R = zeros(1,N);

tic
% Agent tube
for i = 1:N
    tk = t(i);

    % Determine which segment tk belongs to
    seg_idx = find(tk >= breaks(1:end-1) & tk <= breaks(2:end), 1, 'first');
    if isempty(seg_idx)
        seg_idx = M; % last segment safety
    end

    % Segment coefficients from q
    ax = q(seg_idx,1); bx = q(seg_idx,2); cx = q(seg_idx,3);
    ay = q(seg_idx,4); by = q(seg_idx,5); cy = q(seg_idx,6);
    ar = q(seg_idx,7); br = q(seg_idx,8); cr = q(seg_idx,9);

    % Local time scaling within this segment
    t_start = breaks(seg_idx);
    t_end   = breaks(seg_idx+1);
    tau = (tk - t_start) / (t_end - t_start);

    % Evaluate polynomial
    cenx_R(i) = ax + bx*tau + cx*(tau^2);
    ceny_R(i) = ay + by*tau + cy*(tau^2);
    rad_R(i)  = ar + br*tau + cr*(tau^2);
end

xd_R(1) = cenx_R(1);
yd_R(1) = ceny_R(1);
thetad_R(1) = atan2(ceny_R(2)-ceny_R(1), cenx_R(2)-cenx_R(1));

%Trajectory for Agent 1
for i=2:N
    % STT
    ux_R(i) = -10*(xd_R(i-1)-cenx_R(i));
    uy_R(i) = -10*(yd_R(i-1)-ceny_R(i));
    
    % Differential
    vd_R(i) = sqrt(ux_R(i)^2 + uy_R(i)^2);
    wd_R(i) = psi((ux_R(i)^2 + uy_R(i)^2)/1e-6) * ( ux_R(i)*( (uy_R(i)-uy_R(i-1))/dt ) - uy_R(i)*( (ux_R(i)-ux_R(i-1))/dt ) ) / ( ux_R(i)^2 + uy_R(i)^2 );
    xd_R(i) = xd_R(i-1) + dt*vd_R(i)*cos(thetad_R(i-1));
    yd_R(i) = yd_R(i-1) + dt*vd_R(i)*sin(thetad_R(i-1));
    thetad_R(i) = thetad_R(i-1) + dt*wd_R(i);
end

toc
%% Plot the trajectories
figure(1)
hold on;
rectangle('Position',[0.15,0.15,0.4,0.4],'Curvature',1,'FaceColor','b','FaceAlpha',0.5); % Start
rectangle('Position',[2.45,2.45,0.4,0.4],'Curvature',1,'FaceColor','g','FaceAlpha',0.5); % Target
% rectangle('Position',[0.5,0.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5); % Obstacle 1
rectangle('Position',[1.5,1.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5); % Obstacle 2
% rectangle('Position',[0,1.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5); % Obstacle 3
% rectangle('Position',[2,0.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5); % Obstacle 4
plot(cenx_R, ceny_R, 'k--', 'LineWidth',1.5)
plot(xd_R, yd_R, 'b-', 'LineWidth',2)
grid on
%%
function p = psi(v)
    p = zeros(1,length(v));
    
    for i = 1:length(v)
        if v(i) < 1
            p(i) = 0;
        else
            p(i) = 1;
        end
    end
end

%% 3d plot

% theta discretization for circular cross-section
theta = linspace(0, 2*pi, 50);    % finer → smoother tube
N = length(t);

% allocate
Xsurf = zeros(N, length(theta));
Ysurf = zeros(N, length(theta));
Tsurf = zeros(N, length(theta));

% sweep the tube cross-section along the trajectory
for i = 1:N
    Xsurf(i,:) = cenx_R(i) + rad_R(i)*cos(theta);
    Ysurf(i,:) = ceny_R(i) + rad_R(i)*sin(theta);
    Tsurf(i,:) = t(i)*ones(1,length(theta));
end

% create 3D surface
figure; hold on
s = surf(Xsurf, Ysurf, Tsurf);

% surface styling
set(s, 'FaceColor', [0.4 0.6 1],  'FaceAlpha', 0.2, 'EdgeColor', 'none'); 

% plot centerline
plot3(cenx_R, ceny_R, t, 'k', 'LineWidth', 2);

% axis labels and view
xlabel('$x_1$ (m)','Interpreter','latex');
ylabel('$x_2$ (m)','Interpreter','latex');
zlabel('$t$ (s)','Interpreter','latex');
xlim([0 3])
ylim([0 3])
zlim([0 5])

grid on

% optional styling
set(gca, 'YDir', 'reverse','FontSize',12);

%% 2D plots at selected time steps

% Time steps to plot
t_snap = [2, 3, 5];

% Robot color & tube color
tube_color = [0.4 0.6 1];
robot_color = [0 0 0];

% Obstacle parameters
OA_x0 = 0.95; OA_y0 = 0.35; OA_R = 0.25;  % dynamic up
OB_x0 = 1.65; OB_y0 = 2.65; OB_R = 0.25;  % dynamic down
OC_x0 = 0.85; OC_y0 = 0.45; OC_R = 0.25;  % static
OD_x0 = 1.15; OD_y0 = 2.55; OD_R = 0.25;  % static

speed_A = (1.35 - 0.35) / (max(t)-min(t));
speed_B = (2.65 - 1.35) / (max(t)-min(t));

for k = 1:length(t_snap)
    tk = t_snap(k);
    % Find closest index in time array
    [~, idx] = min(abs(t - tk));

    figure; hold on; axis equal;
    plot(cenx_R(1:idx), ceny_R(1:idx), 'Color', tube_color, 'LineWidth', 1.5); % center trajectory

    th = linspace(0, 2*pi, 100);
    Xcirc = cenx_R(idx) + rad_R(idx)*cos(th);
    Ycirc = ceny_R(idx) + rad_R(idx)*sin(th);
    fill(Xcirc, Ycirc, tube_color, 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    rectangle('Position',[0.15,0.15,0.4,0.4],'Curvature',1,'FaceColor','b','FaceAlpha',0.5,'EdgeColor', 'none'); % Start
    rectangle('Position',[2.45,2.45,0.4,0.4],'Curvature',1,'FaceColor','g','FaceAlpha',0.5,'EdgeColor', 'none'); % Target
    rectangle('Position',[1.75,0.75,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5,'EdgeColor', 'none'); % Target
    rectangle('Position',[2.25,1.75,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5,'EdgeColor', 'none'); % Target
    rectangle('Position',[1.75,1.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5,'EdgeColor', 'none'); % Target

    plot(xd_R(idx), yd_R(idx), 'ko', 'MarkerFaceColor', robot_color, 'MarkerSize', 7);

    oxA = OA_x0; oyA = OA_y0 + speed_A*tk;
    oxB = OB_x0; oyB = OB_y0 - speed_B*tk;
    drawObstacle2D(oxA, oyA, OA_R, [1 0 0]);
    drawObstacle2D(oxB, oyB, OB_R, [1 0 0]);
    drawObstacle2D(OC_x0, OC_y0, OC_R, [1 0 0]);
    drawObstacle2D(OD_x0, OD_y0, OD_R, [1 0 0]);

    xlabel('$x_1$ (m)', 'Interpreter','latex');
    ylabel('$x_2$ (m)', 'Interpreter','latex');
    title(sprintf('t = %.1f s', tk));
    set(gca, 'FontSize', 13);
    axis([0 3 0 3]);
    grid on; box on;
    legend('Tube center','Tube boundary','Robot','Obstacle','Location','best');
end

function drawObstacle2D(xc, yc, R, col)
    theta = linspace(0,2*pi,50);
    X = xc + R*cos(theta);
    Y = yc + R*sin(theta);
    fill(X, Y, col, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end