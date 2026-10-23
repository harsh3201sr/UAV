%% =========================================================================
%  MASTER RUN SCRIPT - Quadcopter Drone Simulation (MATLAB R2024a)
%  Executes all 3 parts sequentially and generates comparison report
%% =========================================================================
clc; clear; close all;

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════╗\n');
fprintf('║   QUADCOPTER DRONE SIMULATION - MATLAB R2024a             ║\n');
fprintf('║   6-DOF Dynamics | Battery Model | RL Optimization        ║\n');
fprintf('╚═══════════════════════════════════════════════════════════╝\n\n');

startTime = tic;

%% -------------------------------------------------------------------------
%  CHECK TOOLBOXES
%% -------------------------------------------------------------------------
fprintf('Checking required toolboxes...\n');
tb = ver;
tbNames = {tb.Name};
required = {'Simulink','Reinforcement Learning Toolbox','Control System Toolbox'};
for i = 1:length(required)
    if any(strcmp(tbNames, required{i}))
        fprintf('  [OK] %s\n', required{i});
    else
        fprintf('  [INFO] %s not found - proceeding with MATLAB scripts\n', required{i});
    end
end
fprintf('\n');

%% -------------------------------------------------------------------------
%  PART 1: NORMAL OPERATION
%% -------------------------------------------------------------------------
fprintf('▶ Running PART 1: Normal Battery Operation...\n');
run('Part1_NormalOperation.m');
fprintf('\n');

%% -------------------------------------------------------------------------
%  PART 2: LOW BATTERY
%% -------------------------------------------------------------------------
fprintf('▶ Running PART 2: Low Battery / Power Management...\n');
run('Part2_LowBattery.m');
fprintf('\n');

%% -------------------------------------------------------------------------
%  PART 3: RL OPTIMIZATION
%% -------------------------------------------------------------------------
fprintf('▶ Running PART 3: Reinforcement Learning Optimization...\n');
run('Part3_RLOptimization.m');
fprintf('\n');

%% -------------------------------------------------------------------------
%  LOAD ALL RESULTS
%% -------------------------------------------------------------------------
load('results_part1.mat');
load('results_part2.mat');
load('results_part3.mat');

%% -------------------------------------------------------------------------
%  CROSS-PART COMPARISON FIGURE
%% -------------------------------------------------------------------------
fprintf('Generating cross-part comparison plots...\n');

figure(99);
set(gcf, 'Name', 'ALL PARTS - Comparison', 'Color', 'white', ...
    'Position', [50, 50, 1400, 900]);

% ── SoC Comparison ────────────────────────────────────────────────────────
subplot(2,3,1);
plot(results_p1.t, results_p1.SoC*100, 'b-', 'LineWidth', 2); hold on;
plot(results_p2.t, results_p2.SoC*100, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('SoC (%)');
title('Battery SoC: Part1 vs Part2');
legend('Normal (P1)','Low Batt (P2)','Location','southwest');
ylim([0 110]); grid on;

% ── Voltage Comparison ────────────────────────────────────────────────────
subplot(2,3,2);
plot(results_p1.t, results_p1.Vbatt, 'b-', 'LineWidth', 2); hold on;
plot(results_p2.t, results_p2.Vbatt, 'r-', 'LineWidth', 2);
yline(5.0, 'k--', 'LineWidth', 2, 'Label', '5V limit');
xlabel('Time (s)'); ylabel('Voltage (V)');
title('Battery Voltage: Part1 vs Part2');
legend('Normal (P1)','Low Batt (P2)','Location','southwest');
grid on;

% ── Speed Comparison ──────────────────────────────────────────────────────
subplot(2,3,3);
speed1 = vecnorm(results_p1.vel,2,2);
speed2 = vecnorm(results_p2.vel,2,2);
plot(results_p1.t, speed1, 'b-', 'LineWidth', 1.5); hold on;
plot(results_p2.t, speed2, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Speed Profile: Part1 vs Part2');
legend('Normal','Low Battery','Location','northeast');
grid on;

% ── Power Comparison ──────────────────────────────────────────────────────
subplot(2,3,4);
plot(results_p1.t, results_p1.power, 'b-', 'LineWidth', 1.5); hold on;
plot(results_p2.t, results_p2.power, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Power (W)');
title('Power Consumption: Part1 vs Part2');
legend('Normal','Low Battery'); grid on;

% ── RL Velocity Comparison ────────────────────────────────────────────────
subplot(2,3,5);
legs = categorical({'WP1→2','WP2→3','WP3→4','WP4→5','WP5→6'});
bar(legs, [results_p3.vel_base, results_p3.vel_rl]);
ylabel('Velocity (m/s)');
title('RL Velocity Optimization (Part 3)');
legend('Baseline','RL Optimized');
grid on;

% ── Summary Bar Chart ─────────────────────────────────────────────────────
subplot(2,3,6);
categories = categorical({'Mission Time (s)', 'Energy (Wh)', 'Final SoC (%)'});
% Normalize for display
d = [
    results_p1.t(end),                       results_p2.t(end),                    results_p3.time_rl;
    trapz(results_p1.t,results_p1.power)/3600, trapz(results_p2.t,results_p2.power)/3600, results_p3.energy_rl/3600;
    results_p1.SoC(end)*100,                 results_p2.SoC(end)*100,             results_p3.SoC_rl(end)*100;
];
bar(categories, d);
legend('Part 1','Part 2','Part 3 RL');
title('Overall Metrics Comparison');
grid on;

sgtitle('Drone Simulation - All Parts Comparison', 'FontSize', 14, 'FontWeight', 'bold');

%% -------------------------------------------------------------------------
%  FINAL SUMMARY TABLE
%% -------------------------------------------------------------------------
elapsed = toc(startTime);
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                  FINAL SIMULATION SUMMARY                    ║\n');
fprintf('╠══════════════════════════════════════════════╦═══════════════╣\n');
fprintf('║ Metric                                        ║ Value        ║\n');
fprintf('╠══════════════════════════════════════════════╬═══════════════╣\n');

% Part 1
fprintf('║ [P1] Mission Duration                         ║ %6.1f s      ║\n', results_p1.t(end));
fprintf('║ [P1] Final SoC                                ║ %6.1f %%      ║\n', results_p1.SoC(end)*100);
fprintf('║ [P1] Min Voltage                              ║ %6.2f V      ║\n', min(results_p1.Vbatt));
fprintf('║ [P1] Energy Used                              ║ %6.2f Wh     ║\n', trapz(results_p1.t,results_p1.power)/3600);
fprintf('║ [P1] Voltage Constraint (>=5V)                ║ %-13s ║\n', ...
        ternary(min(results_p1.Vbatt)>=5,'PASS ✓','FAIL ✗'));
fprintf('╠══════════════════════════════════════════════╬═══════════════╣\n');

% Part 2
fprintf('║ [P2] Mission Duration                         ║ %6.1f s      ║\n', results_p2.t(end));
fprintf('║ [P2] Final SoC                                ║ %6.1f %%      ║\n', results_p2.SoC(end)*100);
fprintf('║ [P2] Min Voltage                              ║ %6.2f V      ║\n', min(results_p2.Vbatt));
fprintf('║ [P2] Safety Events                            ║ %6d        ║\n', length(results_p2.safetyEvents));
fprintf('║ [P2] Voltage Constraint (>=5V)                ║ %-13s ║\n', ...
        ternary(min(results_p2.Vbatt)>=5,'PASS ✓','FAIL ✗'));
fprintf('╠══════════════════════════════════════════════╬═══════════════╣\n');

% Part 3
timeSaving = (results_p3.time_base - results_p3.time_rl)/results_p3.time_base*100;
energySaving = (results_p3.energy_base - results_p3.energy_rl)/results_p3.energy_base*100;
fprintf('║ [P3] RL Training Episodes                     ║ %6d        ║\n', results_p3.RL.nEpisodes);
fprintf('║ [P3] Baseline Time                            ║ %6.1f s      ║\n', results_p3.time_base);
fprintf('║ [P3] RL Optimized Time                        ║ %6.1f s      ║\n', results_p3.time_rl);
fprintf('║ [P3] Time Saving (RL vs Baseline)             ║ %+6.1f %%     ║\n', timeSaving);
fprintf('║ [P3] Energy Saving (RL vs Baseline)           ║ %+6.1f %%     ║\n', energySaving);
fprintf('╚══════════════════════════════════════════════╩═══════════════╝\n');
fprintf('\nTotal script execution time: %.1f seconds\n', elapsed);
fprintf('\nAll simulations complete! Check figure windows for plots.\n');

%% =========================================================================
function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end
