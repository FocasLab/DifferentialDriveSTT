clc
clear

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

global edbar
edbar = 1e-2;

kd = 100;
ktheta = 10;
% Initialization
N = length(t);
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
cenx = [ones(length(t),1), t, t.^2, t.^3]*q(2:5);
ceny = [ones(length(t),1), t, t.^2, t.^3]*q(6:9);
rad = q(10);
thetad(1) = 0;
xd(1) = 0.34;
yd(1) = 0.35;
t = t';
cenx = cenx';
ceny = ceny';
rhod_zero = 0.45;
rhod_inf = 0.01;
rhotheta_zero = 0.30;
rhotheta_inf = 0.05;
decay_d = 0.2;
decay_theta = 0.05;
rhod = (rhod_zero - rhod_inf)*exp(-decay_d*t) + rhod_inf;
rhotheta = (rhotheta_zero - rhotheta_inf)*exp(-decay_theta*t) + rhotheta_inf;

tic
for i = 1:N
    ed(i) = norm([xd(i)-cenx(i), yd(i)-ceny(i)])/rad;
    etheta(i) = psi(ed(i)/edbar)*(2/pi)*(atan2(ceny(i)-yd(i),cenx(i)-xd(i))-thetad(i));
    ed(i) = ed(i) / rhod(i);
    etheta(i) = etheta(i) / rhotheta(i);
    eps_d(i) = log((1+ed(i))/(1-ed(i)));
    eps_theta(i) = log((1+etheta(i))/(1-etheta(i)));
    alpha_d(i) = 2/((1-ed(i)^2)*rhod(i)*rad);
    alpha_theta(i) = 4/((1-etheta(i)^2)*pi*rhotheta(i)*ed(i)*rad);
    delta(i) = atan2(ceny(i)-yd(i),cenx(i)-xd(i));
    vd(i) = -kd*(eps_d(i)*alpha_d(i)*cos(delta(i)-thetad(i)) + eps_theta(i)*alpha_theta(i)*sin(delta(i)-thetad(i)));
    wd(i) = ktheta*eps_theta(i)*alpha_theta(i);
    xd(i+1) = xd(i) + dt*vd(i)*cos(thetad(i));
    yd(i+1) = yd(i) + dt*vd(i)*sin(thetad(i));
    thetad(i+1) = thetad(i) + dt*wd(i);
end

toc

figure(1)
hold on;
plot(xd, yd, 'b', 'LineWidth',2)
drawObstacles(start(1), start(2), start(3), [0 0 1])
drawObstacles(target(1), target(2), target(3), [0 1 0])
drawObstacles(obs(:,1), obs(:,2), obs(:,3), [1 0 0])
% rectangle('Position',[0.15,0.15,0.4,0.4],'Curvature',1,'FaceColor','b','FaceAlpha',0.5); % Start
% rectangle('Position',[2.45,2.45,0.4,0.4],'Curvature',1,'FaceColor','g','FaceAlpha',0.5); % Target
% rectangle('Position',[0.5,0.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5); % Obstacle 1
% rectangle('Position',[1.5,1.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5); % Obstacle 2
% rectangle('Position',[0,1.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5); % Obstacle 3
% rectangle('Position',[2,0.5,0.5,0.5],'Curvature',1,'FaceColor','r','FaceAlpha',0.5); % Obstacle 4
% plot(cenx, ceny, 'k--', 'LineWidth',1.5)
legend('Trajectory-STT', 'Start', 'Target', 'Obstacles')
grid on


function act = psi(x)
    global edbar 
    if x < edbar
        act = 0; % Default action
    else
        act = 1; % Action when x is greater than or equal to 0.99
    end
end

function drawObstacles(cx, cy, R, faceColor)
th = linspace(0,2*pi,120);
for i=1:numel(R)
    x = cx(i) + R(i)*cos(th);
    y = cy(i) + R(i)*sin(th);
    fill(x, y, faceColor, 'EdgeColor','none','FaceAlpha',0.5,'LineWidth',1.4);
end
end