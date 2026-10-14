%% Setup
clear; clc; close all;
ratios = [6.0 6.5 7.0 7.4 8.0 8.5 9.0];
outDir = fullfile(pwd, "figures");
[ok, msg] = mkdir(outDir);
if ~ok
    error("Could not create figures folder: %s", msg);
end
%% Stage 0: inspect the data
T = readtable('run_Gf_7.4.csv');
size(T)
head(T)
T.Properties.VariableNames
%% Plot 1
S = readtable('summary.csv');
figure;
plot(S.final_drive, S.top_speed_mph, '-o', 'LineWidth', 1.5)
xlabel('Final drive ratio')
ylabel('Top speed (mph)')
title('Top speed vs. final drive ratio')
grid on
exportgraphics(gcf, 'figures/top_speed_vs_final_drive.png', 'Resolution', 200)
%% Plot 2
ratios = [6.0 6.5 7.0 7.4 8.0 8.5 9.0];
figure; hold on;
for r = ratios
    filename = sprintf('run_Gf_%.1f.csv', r)
    T = readtable(filename);
    plot(T.t, T.v_mph, 'LineWidth', 1.2, 'DisplayName', sprintf('%.1f:1', r))
end
hold off
legend('Location', 'southeast')
xlabel('Time (s)'), ylabel('Speed (mph)'), title('Time Vs. Speed'), grid on
exportgraphics(gcf, 'figures/speed_vs_time.png', 'Resolution', 200)
%% Plot 3
T = readtable("run_Gf_7.4.csv");
figure; hold on
plot(T.v_mph(T.f_trac>0,:), T.f_trac(T.f_trac>0,:), 'DisplayName', 'Tractive force')
plot(T.v_mph(T.f_trac>0,:), T.f_resist(T.f_trac>0,:), 'DisplayName', 'Resistive force')
hold off
legend 
xlabel("Speed (mph)")
ylabel("Force (N)")
title("Force Vs. Speed")
grid on
exportgraphics(gcf, 'figures/force_vs_speed.png', 'Resolution', 200)
%% Plot 4
T = readtable("run_Gf_7.4.csv");
figure
yyaxis left
plot(T.t, T.cvt_ratio);  ylabel("CVT ratio")
yyaxis right
plot(T.t, T.engine_rpm); 
ylabel("Engine speed (rpm)")
xlabel("Time (s)")
title("Engine speed Vs. Time")
grid on
exportgraphics(gcf, 'figures/cvt_and_rpm.png', 'Resolution', 200)