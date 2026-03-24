%% MITL main
clear
clc
% close all

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
549.849550 - rad_enh + 15, 689.175780 + rad_enh;
549.849550 - rad_enh + 15, 680.931640 - rad_enh;
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
322.250000 - rad_enh, 365.863280 - rad_enh;] - [0,5];

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
 pol = augmentPolygon(pol_or, 50);
 %%

% regions of interest
xd0 = sc_cof * [220;-700];
xd1  = sc_cof * [250;-400];
xd2  = sc_cof * [625;-712.5];
xd3 = sc_cof*[125/sc_cof; -75/sc_cof];
xd4 = sc_cof*[200/sc_cof; -130/sc_cof];

% xd2  = [325;-700]/100;

x = [xd0';xd1';xd2';xd3'];

% Scale down and shift up
scale_factor = 1;    % scale to 1/25th size
y_shift = 0;           % shift upward by 12 units

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
figure(2);
clf;
for i = 1:length(pol)
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
% xunit = r * cos(th) + xd3(1);
% yunit = r * sin(th) + xd3(2);
% h = plot(xunit, yunit,'k');
xunit = r * cos(th) + xd4(1);
yunit = r * sin(th) + xd4(2);
h = plot(xunit, yunit,'k');

% grid minor

grid on;

hold on;