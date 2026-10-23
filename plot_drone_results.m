%% =========================================================================
%  PLOT_DRONE_RESULTS.m
%  Shared visualization function for drone simulation results
%  MATLAB R2024a Compatible
%% =========================================================================
function plot_drone_results(res, titleStr, figBase)
%  INPUT:
%   res      - results struct from any part
%   titleStr - figure title prefix
%   figBase  - base figure number

t   = res.t;
pos = res.pos;
vel = res.vel;
SoC = res.SoC;
Vbatt = res.Vbatt;
wp  = res.waypoints;

speed = vecnorm(vel, 2, 2);

%% -----------------------------------------------------------------------
%  FIGURE 1: 3D Trajectory + Waypoints
%% -----------------------------------------------------------------------
figure(figBase*10 + 1);
set(gcf, 'Name', [titleStr ' - 3D Trajectory'], ...
    'Color', 'white', 'Position', [50, 50, 900, 650]);

plot3(pos(:,1), pos(:,2), pos(:,3), 'b-', 'LineWidth', 2); hold on;
scatter3(wp(:,1), wp(:,2), wp(:,3), 120, 'r', 'filled', ...
    'MarkerEdgeColor', 'k');
for i = 1:size(wp,1)
    text(wp(i,1)+0.5, wp(i,2)+0.5, wp(i,3)+0.5, ...
        sprintf('S%d',i), 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
end
scatter3(pos(1,1),   pos(1,2),   pos(1,3),   150, 'g', 'filled', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Start');
scatter3(pos(end,1), pos(end,2), pos(end,3), 150, 'm', 'filled', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'End');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title([titleStr ' - 3D Trajectory']);
legend('Flight Path', 'Waypoints', 'Location', 'best');
grid on; view(45, 30);

%% -----------------------------------------------------------------------
%  FIGURE 2: Position vs Time
%% -----------------------------------------------------------------------
figure(figBase*10 + 2);
set(gcf, 'Name', [titleStr ' - Position vs Time'], ...
    'Color', 'white', 'Position', [80, 80, 1100, 750]);

subplot(3,1,1);
plot(t, pos(:,1), 'b-', 'LineWidth', 1.5); hold on;
yline(wp(end,1), 'r--', 'LineWidth', 1, 'Label', 'Target X');
xlabel('Time (s)'); ylabel('X (m)');
title('X Position vs Time'); grid on;

subplot(3,1,2);
plot(t, pos(:,2), 'g-', 'LineWidth', 1.5); hold on;
yline(wp(end,2), 'r--', 'LineWidth', 1, 'Label', 'Target Y');
xlabel('Time (s)'); ylabel('Y (m)');
title('Y Position vs Time'); grid on;

subplot(3,1,3);
plot(t, pos(:,3), 'r-', 'LineWidth', 1.5); hold on;
yline(wp(end,3), 'k--', 'LineWidth', 1, 'Label', 'Target Z');
xlabel('Time (s)'); ylabel('Z (m)');
title('Z Position (Altitude) vs Time'); grid on;

sgtitle(titleStr, 'FontSize', 13, 'FontWeight', 'bold');

%% -----------------------------------------------------------------------
%  FIGURE 3: Velocity vs Time
%% -----------------------------------------------------------------------
figure(figBase*10 + 3);
set(gcf, 'Name', [titleStr ' - Velocity'], ...
    'Color', 'white', 'Position', [110, 110, 1100, 700]);

subplot(2,2,1);
plot(t, vel(:,1), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Vx (m/s)');
title('X Velocity'); grid on;

subplot(2,2,2);
plot(t, vel(:,2), 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Vy (m/s)');
title('Y Velocity'); grid on;

subplot(2,2,3);
plot(t, vel(:,3), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Vz (m/s)');
title('Z Velocity (Climb Rate)'); grid on;

subplot(2,2,4);
plot(t, speed, 'm-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Total Speed |V|'); grid on;

sgtitle([titleStr ' - Velocity Components'], 'FontSize', 13, 'FontWeight', 'bold');

%% -----------------------------------------------------------------------
%  FIGURE 4: Battery SoC & Voltage
%% -----------------------------------------------------------------------
figure(figBase*10 + 4);
set(gcf, 'Name', [titleStr ' - Battery'], ...
    'Color', 'white', 'Position', [140, 140, 1000, 700]);

subplot(2,1,1);
yyaxis left;
plot(t, SoC*100, 'b-', 'LineWidth', 2);
ylabel('SoC (%)'); ylim([0 110]);
yyaxis right;
plot(t, Vbatt, 'r-', 'LineWidth', 2);
ylabel('Voltage (V)');

% Mark critical levels
hold on;
yline(5.0, 'k--', 'LineWidth', 2, 'Label', 'Hard Min 5V');
yline(6.0, 'Color', [0.8 0.4 0], 'LineStyle', '--', 'Label', 'Preferred 6V');

xlabel('Time (s)');
title('Battery SoC (%) and Voltage vs Time');
legend('SoC (%)','Voltage (V)', 'Location', 'northeast');
grid on;

subplot(2,1,2);
if isfield(res, 'power')
    plot(t, res.power, 'Color', [0.5 0 0.5], 'LineWidth', 1.5);
    ylabel('Power (W)');
    xlabel('Time (s)');
    title('Electrical Power Consumption vs Time');
    grid on;
    % Add energy marker
    E_total = trapz(t, res.power) / 3600;  % Wh
    text(t(end)*0.6, max(res.power)*0.85, ...
        sprintf('Total: %.2f Wh', E_total), 'FontSize', 10, 'Color', 'k');
end

sgtitle([titleStr ' - Battery Status'], 'FontSize', 13, 'FontWeight', 'bold');

%% -----------------------------------------------------------------------
%  FIGURE 5: Summary Dashboard
%% -----------------------------------------------------------------------
figure(figBase*10 + 5);
set(gcf, 'Name', [titleStr ' - Summary Dashboard'], ...
    'Color', 'white', 'Position', [170, 170, 1300, 800]);

% Subplot 1: XY trajectory top view
subplot(2,3,1);
plot(pos(:,1), pos(:,2), 'b-', 'LineWidth', 2); hold on;
scatter(wp(:,1), wp(:,2), 100, 'r', 'filled');
for i = 1:size(wp,1)
    text(wp(i,1)+0.3, wp(i,2)+0.3, sprintf('S%d',i), ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
end
xlabel('X (m)'); ylabel('Y (m)');
title('XY Trajectory (Top View)'); axis equal; grid on;

% Subplot 2: Altitude profile
subplot(2,3,2);
plot(t, pos(:,3), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Profile'); grid on;

% Subplot 3: Speed profile
subplot(2,3,3);
plot(t, speed, 'm-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Speed vs Time'); grid on;

% Subplot 4: Battery SoC
subplot(2,3,4);
area(t, SoC*100, 'FaceColor', [0.2 0.6 1.0], 'FaceAlpha', 0.5);
hold on;
yline(25, 'r--', 'LineWidth', 1.5, 'Label', 'Warning 25%');
yline(15, 'k--', 'LineWidth', 1.5, 'Label', 'Critical 15%');
xlabel('Time (s)'); ylabel('SoC (%)');
title('Battery State of Charge'); ylim([0 110]); grid on;

% Subplot 5: Voltage
subplot(2,3,5);
plot(t, Vbatt, 'r-', 'LineWidth', 2);
hold on;
yline(5.0, 'k--', 'LineWidth', 2, 'Label', 'Min 5V');
yline(6.0, 'b--', 'LineWidth', 1.5, 'Label', 'Pref 6V');
xlabel('Time (s)'); ylabel('Voltage (V)');
title('Battery Voltage'); grid on;

% Subplot 6: Stats text box
subplot(2,3,6);
axis off;
stats_txt = {
    sprintf('=== %s ===', titleStr(1:min(20,end))),
    '',
    sprintf('Duration      : %.1f s', t(end)),
    sprintf('Max Speed     : %.2f m/s', max(speed)),
    sprintf('Avg Speed     : %.2f m/s', mean(speed)),
    '',
    sprintf('Init SoC      : %.1f%%', SoC(1)*100),
    sprintf('Final SoC     : %.1f%%', SoC(end)*100),
    sprintf('Min Voltage   : %.2f V', min(Vbatt)),
    sprintf('Max Voltage   : %.2f V', max(Vbatt)),
    '',
};
if isfield(res, 'power')
    E_wh = trapz(t, res.power)/3600;
    Pavg = mean(res.power);
    stats_txt{end+1} = sprintf('Avg Power     : %.1f W', Pavg);
    stats_txt{end+1} = sprintf('Energy Used   : %.2f Wh', E_wh);
end
stats_txt{end+1} = '';
stats_txt{end+1} = sprintf('V >= 5V       : %s', ...
    ternary(min(Vbatt)>=5.0,'PASS ✓','FAIL ✗'));

text(0.05, 0.95, stats_txt, 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'FontSize', 10, ...
    'FontName', 'Courier', 'Interpreter', 'none');

sgtitle([titleStr ' - Summary Dashboard'], 'FontSize', 14, 'FontWeight', 'bold');

end

%% -----------------------------------------------------------------------
function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end
