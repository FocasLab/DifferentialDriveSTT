%% MITL main
clear
clc
clf
%% PLOT ARENA
sc_cof = 40/100;% 
sc_cof_ro = sc_cof;
%sc_cof = (sc_cof + radiu)/sc_cof * sc_cof;
rad_enh = 1.5/sc_cof; %this is needed for volume absorption,  0 for original space, .75 for single, 1.5 for coop
pol_or = {...
   sc_cof*[73.220703 + rad_enh, 292.304690 + rad_enh;
73.220703 + rad_enh, 444.078120 - rad_enh;
186.035160 + rad_enh, 444.078120 - rad_enh;
186.035160 + rad_enh, 452.800780 + rad_enh;
73.220703 + rad_enh, 452.800780 + rad_enh;
73.220703 + rad_enh, 553.345700 - rad_enh;
167.533200 + rad_enh, 553.345700 - rad_enh;
167.533200 + rad_enh, 560.923830 + rad_enh; 
73.269531 + rad_enh, 560.923830 + rad_enh;
73.269531 + rad_enh, 759.933590 - rad_enh;
229.419920 - rad_enh, 759.802730 - rad_enh;
229.419920 - rad_enh, 734.056640 - rad_enh;
250.312500 - rad_enh, 734.056640 - rad_enh;
250.312500 - rad_enh, 615.958980 + rad_enh;
229.419920 - rad_enh, 615.958980 + rad_enh;
229.419920 - rad_enh, 592.255860 - rad_enh;
250.312500 - rad_enh, 592.255860 - rad_enh;
250.312500 - rad_enh, 560.923830 + rad_enh;
207.611330 - rad_enh, 560.923830 + rad_enh;
207.611330 - rad_enh, 553.345700 - rad_enh;
299.400390 + rad_enh, 553.345700 - rad_enh;
299.462890 + rad_enh, 560.923830 + rad_enh;
257.025390 + rad_enh, 560.923830 + rad_enh;
257.025390 + rad_enh, 734.056640 - rad_enh;
277.226560 + rad_enh, 734.056640 - rad_enh;
277.226560 + rad_enh, 760.035160 - rad_enh;
376.623050 - rad_enh, 760.035160 - rad_enh;
376.623050 - rad_enh, 560.923830 + rad_enh;
334.945310 - rad_enh, 560.923830 + rad_enh;
334.957010 - rad_enh, 553.345700 - rad_enh;
428.542950 + rad_enh, 553.345700 - rad_enh;
428.570250 + rad_enh, 560.923830 + rad_enh;
383.619080 + rad_enh, 560.923830 + rad_enh;
383.619080 + rad_enh, 760.056640 - rad_enh;
505.615170 - rad_enh, 759.802730 - rad_enh;
505.615170 - rad_enh, 560.923830 + rad_enh;
465.066350 - rad_enh, 560.923830 + rad_enh;
465.216740 - rad_enh, 553.345700 - rad_enh;
512.826110 + rad_enh, 553.345700 - rad_enh;
512.826110 + rad_enh, 759.802730 - rad_enh;
630.222600 - rad_enh, 759.802730 - rad_enh;
630.431580 - rad_enh, 736.000000 - rad_enh;
670.874940 - rad_enh, 736.509770 - rad_enh;
670.874940 - rad_enh, 689.175780 + rad_enh;
549.849550 - rad_enh + 30, 689.175780 + rad_enh;
549.849550 - rad_enh + 30, 680.931640 - rad_enh;
670.874940 - rad_enh, 680.931640 - rad_enh;
670.874940 - rad_enh, 452.685550 + rad_enh;
549.849550 - rad_enh, 452.685550 + rad_enh;
549.849550 - rad_enh, 444.173830 - rad_enh;
670.820250 - rad_enh, 444.173830 - rad_enh;
670.486270 - rad_enh, 292.304690 + rad_enh;
633.214780 + rad_enh, 292.304690 + rad_enh;
633.214780 + rad_enh, 327.222660 + rad_enh;
596.972600 - rad_enh, 327.222660 + rad_enh;
596.732360 - rad_enh, 292.304690 + rad_enh;
558.130800 + rad_enh, 292.304690 + rad_enh;
558.130800 + rad_enh, 405.199220 + rad_enh;
549.849550 - rad_enh, 405.199220 + rad_enh;
549.849550 - rad_enh, 375.789060 + rad_enh;
525.013610 - rad_enh, 375.789060 + rad_enh;
525.013610 - rad_enh, 365.863280 - rad_enh;
549.849550 - rad_enh, 365.863280 - rad_enh;
549.849550 - rad_enh, 292.761720 + rad_enh;
418.234320 + rad_enh, 292.761720 + rad_enh;
418.234320 + rad_enh, 365.863280 - rad_enh;
441.187440 + rad_enh, 365.863280 - rad_enh;
441.187440 + rad_enh, 375.789060 + rad_enh;
357.140560 - rad_enh, 375.789060 + rad_enh;
357.140560 - rad_enh, 365.863280 - rad_enh;
410.140560 - rad_enh, 365.863280 - rad_enh;
410.140560 - rad_enh, 292.304690 + rad_enh;
260.095640 + rad_enh, 292.304690 + rad_enh;
260.095640 + rad_enh, 327.222660 + rad_enh;
227.687440 - rad_enh, 327.222660 + rad_enh;
227.699140 - rad_enh, 292.304690 + rad_enh;
73.220703 + rad_enh, 292.304690 + rad_enh;];
sc_cof*[
117.191410 - rad_enh, 337.005860 - rad_enh;
165.244140 + rad_enh, 337.005860 - rad_enh;
165.244140 + rad_enh, 344.332030 + rad_enh;
117.191410 - rad_enh, 344.332030 + rad_enh;
117.191410 - rad_enh, 337.005860 - rad_enh;];
sc_cof*[
322.250000 - rad_enh, 365.863280 - rad_enh;
329.810550 + rad_enh, 365.863280 - rad_enh;
329.810550 + rad_enh, 452.800780 + rad_enh;
322.250000 - rad_enh, 452.800780 + rad_enh;
322.250000 - rad_enh, 365.863280 - rad_enh;] - [0,10];
sc_cof*[
461.404300 - rad_enh, 365.863280 - rad_enh;
505.625000 + rad_enh, 365.863280 - rad_enh;
505.625000 + rad_enh, 375.789060 + rad_enh;
461.404300 - rad_enh, 375.789060 + rad_enh;
461.404300 - rad_enh, 365.863280 - rad_enh;];
sc_cof*[
117.191410 - rad_enh, 374.441410 - rad_enh;
165.244140 + rad_enh, 374.441410 - rad_enh;
165.244140 + rad_enh, 381.769530 + rad_enh;
117.191410 - rad_enh, 381.769530 + rad_enh;
117.191410 - rad_enh, 374.441410 - rad_enh;];
sc_cof*[
357.140620 - rad_enh, 399.291020 - rad_enh;
513.333980 + rad_enh, 399.291020 - rad_enh;
513.333980 + rad_enh, 492.041020 + rad_enh;
504.876950 - rad_enh, 492.041020 + rad_enh;
504.876950 - rad_enh, 408.652340 + rad_enh;
357.140620 - rad_enh, 408.652340 + rad_enh;
357.140620 - rad_enh, 399.291020 - rad_enh;];
sc_cof*[
430.910160 - rad_enh, 439.988280 - rad_enh;
454.955080 + rad_enh, 439.988280 - rad_enh;
454.955080 + rad_enh, 463.671880 + rad_enh;
430.910160 - rad_enh, 463.671880 + rad_enh;
430.910160 - rad_enh, 439.988280 - rad_enh;];
sc_cof*[
228.511720 - rad_enh, 444.078120 - rad_enh;
285.060550 + rad_enh, 444.078120 - rad_enh;
285.060550 + rad_enh, 452.800780 + rad_enh;
228.511720 - rad_enh, 452.800780 + rad_enh;
228.511720 - rad_enh, 444.078120 - rad_enh;];
sc_cof*[
582.216800 - rad_enh, 494.183590 - rad_enh;
624.406250 + rad_enh, 494.183590 - rad_enh;
624.406250 + rad_enh, 635.808590 + rad_enh;
582.216800 - rad_enh, 635.808590 + rad_enh;
582.216800 - rad_enh, 494.183590 - rad_enh;];
sc_cof*[
121.238280 - rad_enh, 632.779300 - rad_enh;
194.796880 + rad_enh, 632.779300 - rad_enh;
194.796880 + rad_enh, 699.769530 + rad_enh;
121.238280 - rad_enh, 699.769530 + rad_enh;
121.238280 - rad_enh, 632.779300 - rad_enh;] - [5,0];
sc_cof*[
430.910160 - rad_enh, 657.273440 - rad_enh;
454.955080 + rad_enh, 657.273440 - rad_enh;
454.955080 + rad_enh, 680.976560 + rad_enh;
430.910160 - rad_enh, 680.976560 + rad_enh;
430.910160 - rad_enh, 657.273440 - rad_enh;];
};
for i = 1 : size(pol_or, 1)
    pol_or{i}(:,2) = -1*pol_or{i}(:,2);
    pol_or{i}(:,:) = pol_or{i}(:,:);
end
%
 pol = augmentPolygon(pol_or, 50); % Assuming augmentPolygon is a user-defined function
 %%
% regions of interest
xd0 = sc_cof * [220;-700];
xd1  = sc_cof * [250;-400];
xd2  = sc_cof * [625;-712.5];
xd3 = sc_cof * [525;-525];
xd4 = sc_cof*[200/sc_cof; -130/sc_cof];
% xd2  = [325;-700]/100;
x_rois = [xd0';xd1';xd2';xd3']; % Renamed 'x' to 'x_rois' to avoid conflict
% Scale down and shift up
scale_factor = 1/25;    % scale to 1/25th size
y_shift = 12;           % shift upward by 12 units
for i = 1:length(pol_or)
    pol_or{i} = pol_or{i} * scale_factor;     % scale polygon
    pol_or{i}(:,2) = pol_or{i}(:,2) + y_shift; % shift upward
end
% Also scale and shift regions of interest
xd0 = xd0 * scale_factor; xd0(2) = xd0(2) + y_shift;
xd1 = xd1 * scale_factor; xd1(2) = xd1(2) + y_shift;
xd2 = xd2 * scale_factor; xd2(2) = xd2(2) + y_shift;
xd3 = xd3 * scale_factor; xd3(2) = xd3(2) + y_shift;
xd4 = xd4 * scale_factor; xd4(2) = xd4(2) + y_shift;
%
figure(1) 
clf;
subplot(2, 3, [3,6]);
for i = 1:length(pol_or) % Use pol_or, 'pol' is not initialized as a cell array
   plot(polyshape(pol_or{i}));
   hold on;
 end
box on;
axis equal;
th = 0:pi/50:2*pi;
r = ( 4 + rad_enh*sc_cof )* scale_factor;    %+.75 or 1.5 for single and multi
xunit = r * cos(th) + xd0(1);
yunit = r * sin(th) + xd0(2);
h = plot(xunit, yunit,'k');
xunit = r * cos(th) + xd1(1);
yunit = r * sin(th) + xd1(2);
h = plot(xunit, yunit,'k');
xunit = r * cos(th) + xd2(1);
yunit = r * sin(th) + xd2(2);
h = plot(xunit, yunit,'k');
xunit = r * cos(th) + xd3(1);
yunit = r * sin(th) + xd3(2);
h = plot(xunit, yunit,'k');
xunit = r * cos(th) + xd4(1);
yunit = r * sin(th) + xd4(2);
h = plot(xunit, yunit,'k');
% grid minor
grid on;
% xticks(0:0.4:12)
% yticks(0:0.4:12)
hold on;
%% PLOT TRAJECTORY
q = readmatrix("coefficients_Final.csv");
q1 = q(1:10,2);
q2 = q(12:25,2);
q3 = q(27:36,2);

% Simulation & System Parameters
tsc = 10;
dt = 1e-4;      % Time step [s] (Increased precision)
tf = 10*tsc;        % Final time [s]
tim = 0:dt:tf;
N = length(tim);

% System State (Unicycle Model): [x; y; theta]
x0 = [3.4; 0.8; pi/2]; % Initial state [x; y; theta]
x = zeros(3, N);
x(:,1) = x0;

% Pre-allocate control and error history for plotting
vd = zeros(1,N);
wd = zeros(1,N);
ed_history = zeros(1, N);
eth_history = zeros(1, N);
rhod_history = zeros(1, N);
rhoth_history = zeros(1, N);
cenx_history = zeros(1, N);
ceny_history = zeros(1, N);

% --- Calculate initial reference center (at t=0.5) for history ---
t_start = tim(1);
qnow_start = q1;
Coeffx_start = qnow_start(1:5);
Coeffy_start = qnow_start(6:10);
t_vec_start = [1; t_start; t_start^2; t_start^3; t_start^4];
cen_start = [Coeffx_start' * t_vec_start; Coeffy_start' * t_vec_start];
cenx_history(1) = cen_start(1);
ceny_history(1) = cen_start(2);

% --- Main Simulation Loop ---
fprintf('Starting simulation...\n');

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!  TODO: Set the STT radius 'rad' parameter correctly.        !!!
% !!!  This was q(10) in the other script. It is essential.       !!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
rad = 1.0; % <-- PLACEHOLDER VALUE. SET THIS.

t_vec1 = [1; tim(1); tim(1)^2; tim(1)^3; tim(1)^4];
t_vec2 = [1; tim(2); tim(2)^2; tim(2)^3; tim(2)^4];
x(1,i) = cen_start(1);
x(2,i) = cen_start(2);
x(3,1) = atan2( q1(6:10)'*(t_vec2-t_vec1), q1(1:5)'*(t_vec2-t_vec1) );

tic
for i = 2:N
    t = tim(i);
    % Select appropriate trajectory segment based on time
    if t < 4*tsc
        qnow = q1;
        Coeffx = qnow(1:5);
        Coeffy = qnow(6:10);
        t_vec = [1; t/tsc; (t/tsc)^2; (t/tsc)^3; (t/tsc)^4];
    elseif t >= 4*tsc && t < 7*tsc
        qnow = q2;
        Coeffx = qnow(1:7);
        Coeffy = qnow(8:14);
        t_vec = [1; t/tsc; (t/tsc)^2; (t/tsc)^3; (t/tsc)^4; (t/tsc)^5; (t/tsc)^6];
    else % t >= 7*tsc
        qnow = q3;
        Coeffx = qnow(1:5);
        Coeffy = qnow(6:10);
        t_vec = [1; t/tsc; (t/tsc)^2; (t/tsc)^3; (t/tsc)^4];
    end
    % --- Define Time-Varying Safe Set Center ---
    cen = [Coeffx' * t_vec; Coeffy' * t_vec];
    
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % --- START: STT Funnel Controller (from Script 2) ---
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Controller Gains
    kd = -1.0;
    kth = 1.0;
    
    % Funnel Functions
    rhod = (0.4 - 0.05) * exp(-0.15*t) + 0.05;
    rhoth = (0.4 - 0.04) * exp(-0.5*t) + 0.04;
    
    % Get previous state
    current_x = x(1, i-1);
    current_y = x(2, i-1);
    current_theta = x(3, i-1);
    
    % Target center
    target_x = cen(1);
    target_y = cen(2);

    % Error Calculations
    dist_error_mag = sqrt((current_x - target_x)^2 + (current_y - target_y)^2);
    psi_angle = unwrap(atan2((target_y - current_y), (target_x - current_x)));
    
    % Normalized errors e_d and e_theta
    ed = dist_error_mag / rad; 
    eth = psi_smooth(ed / 1e-2) * (2/pi) * (psi_angle - current_theta); 
    
    % Normalized funnel errors hed and heth
    hed = clip(ed / rhod, -0.999, 0.999);
    heth = clip(eth / rhoth, -0.999, 0.999);
    
    % Transformed Errors
    epsd = log((1 + hed) / (1 - hed));
    epsth = log((1 + heth) / (1 - heth));
    
    % Alpha terms
    alphad = 2 / (1 - hed^2) / rhod / rad;
    alphath = 2 / (1 - heth^2) * (2 / pi) / rhoth / ed / rad;
    if isnan(alphath) || isinf(alphath) % Add safety check for ed -> 0
        alphath = 0;
    end
    
    % Control Laws (Calculate v and w)
    v = kd * (epsd * alphad * cos(psi_angle - current_theta) - epsth * alphath * sin(psi_angle - current_theta));
    w = kth * epsth * alphath;
    
    % Store control inputs
    vd(i) = v;
    wd(i) = w;

    % Store history for plotting
    ed_history(i) = ed;
    eth_history(i) = eth;
    rhod_history(i) = rhod;
    rhoth_history(i) = rhoth;
    cenx_history(i) = target_x;
    ceny_history(i) = target_y;
    
    % Store initial funnel values for t(1)
    if i==2
        t_prev = tim(i-1);
        rhod_history(i-1) = (0.5 - 0.1) * exp(-2*t_prev) + 0.1;
        rhoth_history(i-1) = (0.1 - 0.01) * exp(-2*t_prev) + 0.01;
    end

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % --- END: STT Funnel Controller ---
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % System Dynamics (Unicycle Model Update)
    x(1,i) = x(1,i-1) + dt * v * cos(current_theta);
    x(2,i) = x(2,i-1) + dt * v * sin(current_theta);
    x(3,i) = x(3,i-1) + dt * w;
    
    % STT
    ux = -10*(x(1,i-1)-cen(1));
    uy = -10*(x(2,i-1)-cen(2));
    u = [ux; uy; 0];
    
    x(:,i) = x(:,i-1) + dt * u;
    x(3,i) = atan2( ceny_history(i)-ceny_history(i-1), cenx_history(i)-cenx_history(i-1) );
    x(3,i) = psi_angle;
end

toc
fprintf('Simulation finished. Generating plots...\n');

%% PLOTTING (Combined Style)

% --- Plot 1: 2D Arena and Trajectory ---
figure(1) 
subplot(2, 3, [3,6]);
% Arena is already plotted, 'hold on' is active
% plot(cenx_history, ceny_history, 'k--', 'LineWidth', 1.5, 'DisplayName', 'STT Center $c$(t)');
plot(x(1,1*tsc:end), x(2,1*tsc:end), 'k-', 'LineWidth', 2, 'DisplayName', 'Robot Path');
% legend('Interpreter', 'latex', 'Location', 'northwest');
xlim([1.1,10.8])
ylim([-0.2,7.4])
set(gca,'FontSize',16)
xlabel('$x_1$ (m)','Interpreter','latex','FontSize',18)
ylabel('$x_2$ (m)','Interpreter','latex','FontSize',18)

% --- Plot 2: 3D STT and Error Plots (from Script 2) ---
figure(1);
% Plot 3D Tube
subplot(2, 3, [1,4]);
hold on;
grid on;
% Plot a subset of the tube to avoid overloading the plot
plot_step = max(1, floor(length(tim) / 100)); % Show ~100 tube slices
for i=1:plot_step:length(tim) 
    % Generate circle points
    thetaSTT = linspace(0, 2*pi, 50); % 50 points per circle
    xSTT = cenx_history(i) + rad*cos(thetaSTT);
    ySTT = ceny_history(i) + rad*sin(thetaSTT);
    tSTT = tim(i)*ones(size(thetaSTT));
    fill3(xSTT, ySTT, tSTT, 'b', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
end
% Plot 3D robot path
plot3(x(1,:), x(2,:), tim, 'k-','LineWidth',2) 
xlabel('$x_1$ (m)','Interpreter','latex','FontSize',18)
ylabel('$x_2$ (m)','Interpreter','latex','FontSize',18)
zlabel('$t$ (s)','Interpreter','latex','FontSize',18)
set(gca,'FontSize',16)
view([-25,15]);
hold off;

% Plot distance error and funnel
subplot(2, 3, 2);
hold on;
grid on;
plot(tim(1*tsc:end), ed_history(1*tsc:end), 'k-', 'LineWidth', 2, 'DisplayName', '$e_d(t)$');
plot(tim(1*tsc:end), rhod_history(1*tsc:end), 'b--', 'LineWidth', 1.5, 'DisplayName', '$\rho_d(t)$');
plot(tim(1*tsc:end), -rhod_history(1*tsc:end), 'b--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
legend('Interpreter', 'latex');
xlabel('$t$ (s)','Interpreter','latex','FontSize',18)
ylabel('$e_d$ (m)','Interpreter','latex','FontSize',18)
xlim([tim(1), tf]) % Use simulation time
set(gca,'FontSize',18);
hold off;

% Plot orientation error and funnel
subplot(2, 3, 5);
hold on;
grid on;
plot(tim(1*tsc:end), eth_history(1*tsc:end), 'k-', 'LineWidth', 2, 'DisplayName', '$e_\theta(t)$');
plot(tim(1*tsc:end), rhoth_history(1*tsc:end), 'b--', 'LineWidth', 1.5, 'DisplayName', '$\rho_\theta(t)$');
plot(tim(1*tsc:end), -rhoth_history(1*tsc:end), 'b--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
legend('Interpreter', 'latex');
xlabel('$t$ (s)','Interpreter','latex','FontSize',18)
ylabel('$e_\theta$ (rad)','Interpreter','latex','FontSize',18)
xlim([tim(1), tf]) % Use simulation time
set(gca,'FontSize',18);
hold off;

fprintf('Plotted 3D and error graphs on Figure 2.\n');

%% Helper functions (from Script 2)
function p = psi_smooth(v)
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

function v_clipped = clip(v, min_val, max_val)
    v_clipped = max(min(v, max_val), min_val);
end