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
global edbar
edbar = 1e-1;

kd = 0.1;
ktheta = 10.1;
% Initialization
ed = zeros(1,N);
etheta = zeros(1,N);
eps_d = zeros(1,N);
eps_theta = zeros(1,N);
alpha_d = zeros(1,N);
alpha_theta = zeros(1,N);
delta = zeros(1,N);

ux = zeros(1,N);
uy = zeros(1,N);
xd = zeros(1,N+1);
yd = zeros(1,N+1);
thetad = zeros(1,N+1);
vd = zeros(1,N);
wd = zeros(1,N);

rhod_zero = 0.45;
rhod_inf = 0.01;
rhotheta_zero = 1.5;
rhotheta_inf = 0.1;
decay_d = 0.2/100;
decay_theta = 0.05/10;
rhod = (rhod_zero - rhod_inf)*exp(-decay_d*t) + rhod_inf;
rhotheta = (rhotheta_zero - rhotheta_inf)*exp(-decay_theta*t) + rhotheta_inf;

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
    cenx(i) = ax + bx*tau + cx*(tau^2);
    ceny(i) = ay + by*tau + cy*(tau^2);
    rad(i)  = ar + br*tau + cr*(tau^2);
end

xd(1) = cenx(1);
yd(1) = ceny(1)-0.01;
thetad(1) = 0;

%Trajectory for robot
dm = -0.01;
for i = 1:N-1
    ed(i) = norm([xd(i)-cenx(i), yd(i)-ceny(i)])/rad(i);
    etheta(i) = psi(ed(i))*(2/pi)*(atan2(ceny(i)-yd(i),cenx(i)-xd(i))-thetad(i));
    
    ed(i) = min(max(ed(i) / rhod(i),-1+dm),1-dm);
    etheta(i) = min(max(etheta(i) / rhotheta(i),-1+dm),1-dm);

    eps_d(i) = log((1+ed(i))/(1-ed(i)));
    eps_theta(i) = log((1+etheta(i))/(1-etheta(i)));

    alpha_d(i) = 2/((1-ed(i)^2)*rhod(i)*rad(i));
    alpha_theta(i) = 4/((1-etheta(i)^2)*pi*rhotheta(i)*ed(i)*rad(i));
    delta(i) = atan2(ceny(i)-yd(i),cenx(i)-xd(i));
    
    vd(i) = kd*(eps_d(i)*alpha_d(i)*cos(delta(i)-thetad(i)) - eps_theta(i)*alpha_theta(i)*sin(delta(i)-thetad(i)));
    wd(i) = ktheta*eps_theta(i)*alpha_theta(i);

    xd(i+1) = xd(i) + dt*vd(i)*cos(thetad(i));
    yd(i+1) = yd(i) + dt*vd(i)*sin(thetad(i));
    thetad(i+1) = thetad(i) + dt*wd(i);
    
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
plot(cenx, ceny, 'k--', 'LineWidth',1.5)
plot(xd(1:end-1), yd(1:end-1), 'b-', 'LineWidth',2)
grid on
%%
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
    Xsurf(i,:) = cenx(i) + rad(i)*cos(theta);
    Ysurf(i,:) = ceny(i) + rad(i)*sin(theta);
    Tsurf(i,:) = t(i)*ones(1,length(theta));
end

% create 3D surface
figure; hold on
s = surf(Xsurf, Ysurf, Tsurf);

% surface styling
set(s, 'FaceColor', [0.4 0.6 1],  'FaceAlpha', 0.2, 'EdgeColor', 'none'); 

% plot centerline
plot3(cenx, ceny, t, 'k', 'LineWidth', 2);

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
    plot(cenx(1:idx), ceny(1:idx), 'Color', tube_color, 'LineWidth', 1.5); % center trajectory

    th = linspace(0, 2*pi, 100);
    Xcirc = cenx(idx) + rad(idx)*cos(th);
    Ycirc = ceny(idx) + rad(idx)*sin(th);
    fill(Xcirc, Ycirc, tube_color, 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    rectangle('Position',[0.15,0.15,0.4,0.4],'Curvature',1,'FaceColor','b','FaceAlpha',0.5,'EdgeColor', 'none'); % Start
    rectangle('Position',[2.45,2.45,0.4,0.4],'Curvature',1,'FaceColor','g','FaceAlpha',0.5,'EdgeColor', 'none'); % Target
    rectangle('Position',[1.75,0.75,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5,'EdgeColor', 'none'); % Target
    rectangle('Position',[2.25,1.75,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5,'EdgeColor', 'none'); % Target
    rectangle('Position',[1.75,1.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5,'EdgeColor', 'none'); % Target

    plot(xd(idx), yd(idx), 'ko', 'MarkerFaceColor', robot_color, 'MarkerSize', 7);

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