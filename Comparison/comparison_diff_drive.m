clc;
clear all;
tic
%% Read Data
format long
q = readmatrix("comparison.xlsx");
start = [0.35 0.35 0.2];
target = [2.65 2.65 0.2];
obs = [0.75 0.75 0.25;
       1.75 1.75 0.25;
       0.25 1.75 0.25;
       2.25 0.75 0.25];
%%
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
%%
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
%% Plotting
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
toc
%%
function p = psi(v)
    p = zeros(1,length(v));
    
    % for i = 1:length(v)
    %     p(i) = 0.5 + 0.5*tanh(10*(v(i)-1e-6));
    % end
    for i = 1:length(v)
        if v(i) < 1
            p(i) = 0;
        else
            p(i) = 1;
        end
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
 