%% =========================================================================
%  VALIDATE_DRONE_SIM.m
%  Post-simulation validation, error checking, and constraint verification
%  MATLAB R2024a Compatible
%% =========================================================================
clc;

fprintf('===========================================\n');
fprintf(' DRONE SIMULATION VALIDATION SUITE\n');
fprintf('===========================================\n\n');

%% -------------------------------------------------------------------------
%  LOAD RESULTS
%% -------------------------------------------------------------------------
resultsExist = all([exist('results_part1.mat','file'), ...
                    exist('results_part2.mat','file'), ...
                    exist('results_part3.mat','file')]);

if ~resultsExist
    error(['Results files not found. Please run RunAll_DroneSimulation.m first.\n' ...
           'Command: run(''RunAll_DroneSimulation.m'')']);
end

load('results_part1.mat');
load('results_part2.mat');
load('results_part3.mat');

%% -------------------------------------------------------------------------
%  DEFINE VALIDATION CHECKS
%% -------------------------------------------------------------------------
tests = struct();
testCount = 0;

%% PART 1 CHECKS
fprintf('--- PART 1: Normal Operation ---\n');

% Check 1: Mission completed (final position near last waypoint)
wp_final = results_p1.waypoints(end,:);
pos_final_p1 = results_p1.pos(end,:);
dist_to_target_p1 = norm(pos_final_p1 - wp_final);
testCount = testCount+1;
tests(testCount).name = '[P1] Mission completed (reached WP6)';
tests(testCount).pass = dist_to_target_p1 < 2.0;
tests(testCount).value = sprintf('%.2f m from target', dist_to_target_p1);

% Check 2: Voltage never below 5V
minV_p1 = min(results_p1.Vbatt);
testCount = testCount+1;
tests(testCount).name = '[P1] Battery voltage >= 5V always';
tests(testCount).pass = minV_p1 >= 5.0;
tests(testCount).value = sprintf('Min = %.2f V', minV_p1);

% Check 3: No ground penetration
minAlt_p1 = min(results_p1.pos(:,3));
testCount = testCount+1;
tests(testCount).name = '[P1] Altitude >= 0 (no ground penetration)';
tests(testCount).pass = minAlt_p1 >= -0.01;
tests(testCount).value = sprintf('Min alt = %.3f m', minAlt_p1);

% Check 4: Velocity stability (no excessive oscillations)
speed_p1 = vecnorm(results_p1.vel,2,2);
maxSpeed_p1 = max(speed_p1);
testCount = testCount+1;
tests(testCount).name = '[P1] Max speed < 15 m/s (no instability)';
tests(testCount).pass = maxSpeed_p1 < 15.0;
tests(testCount).value = sprintf('Max = %.2f m/s', maxSpeed_p1);

% Check 5: SoC stays positive
minSoC_p1 = min(results_p1.SoC);
testCount = testCount+1;
tests(testCount).name = '[P1] SoC stays positive';
tests(testCount).pass = minSoC_p1 >= 0;
tests(testCount).value = sprintf('Min SoC = %.1f%%', minSoC_p1*100);

% Check 6: Attitude angles bounded (roll/pitch < 30 deg)
maxRoll_p1  = max(abs(results_p1.ang(:,1))) * 180/pi;
maxPitch_p1 = max(abs(results_p1.ang(:,2))) * 180/pi;
testCount = testCount+1;
tests(testCount).name = '[P1] Attitude angles bounded (|roll|,|pitch| < 30 deg)';
tests(testCount).pass = maxRoll_p1 < 30 && maxPitch_p1 < 30;
tests(testCount).value = sprintf('Max roll=%.1f° pitch=%.1f°', maxRoll_p1, maxPitch_p1);

%% PART 2 CHECKS
fprintf('\n--- PART 2: Low Battery Operation ---\n');

% Check 7: Hard voltage limit never violated
minV_p2 = min(results_p2.Vbatt);
testCount = testCount+1;
tests(testCount).name = '[P2] Voltage NEVER drops below 5V (hard constraint)';
tests(testCount).pass = minV_p2 >= 5.0;
tests(testCount).value = sprintf('Min = %.4f V', minV_p2);

% Check 8: Mission still completed
pos_final_p2 = results_p2.pos(end,:);
dist_to_target_p2 = norm(pos_final_p2 - wp_final);
testCount = testCount+1;
tests(testCount).name = '[P2] Mission completed despite low battery';
tests(testCount).pass = dist_to_target_p2 < 3.0;
tests(testCount).value = sprintf('%.2f m from target', dist_to_target_p2);

% Check 9: Speed reduction occurred when SoC was low
speedFactor_min = min(results_p2.speedFactor);
testCount = testCount+1;
tests(testCount).name = '[P2] Power management activated (speed reduced)';
tests(testCount).pass = speedFactor_min < 1.0;
tests(testCount).value = sprintf('Min speed factor = %.0f%%', speedFactor_min*100);

% Check 10: Battery SoC started low (model correctly initialized)
testCount = testCount+1;
tests(testCount).name = '[P2] Battery correctly initialized at low SoC (<50%)';
tests(testCount).pass = results_p2.SoC(1) < 0.50;
tests(testCount).value = sprintf('Initial SoC = %.1f%%', results_p2.SoC(1)*100);

% Check 11: Voltage monotonically decreasing trend (degradation)
vbatt2 = results_p2.Vbatt;
trend = polyfit(1:length(vbatt2), vbatt2', 1);
testCount = testCount+1;
tests(testCount).name = '[P2] Battery voltage shows discharge trend';
tests(testCount).pass = trend(1) <= 0;
tests(testCount).value = sprintf('Trend slope = %.4f V/step', trend(1));

% Check 12: Safety events logged
testCount = testCount+1;
tests(testCount).name = '[P2] Safety events properly logged';
tests(testCount).pass = ~isempty(results_p2.safetyEvents);
tests(testCount).value = sprintf('%d events logged', length(results_p2.safetyEvents));

%% PART 3 CHECKS
fprintf('\n--- PART 3: RL Optimization ---\n');

% Check 13: Q-table has converged (non-zero values)
Qmax = max(abs(results_p3.Q(:)));
testCount = testCount+1;
tests(testCount).name = '[P3] Q-table populated (learning occurred)';
tests(testCount).pass = Qmax > 0;
tests(testCount).value = sprintf('Max |Q| = %.2f', Qmax);

% Check 14: RL agent achieves reasonable success rate
successRate = mean(results_p3.episodeSuccess(end-min(99,length(results_p3.episodeSuccess)-1):end))*100;
testCount = testCount+1;
tests(testCount).name = '[P3] Final success rate >= 70%';
tests(testCount).pass = successRate >= 70;
tests(testCount).value = sprintf('%.1f%%', successRate);

% Check 15: RL is not grossly worse than baseline
timeRatio = results_p3.time_rl / results_p3.time_base;
testCount = testCount+1;
tests(testCount).name = '[P3] RL time not more than 2x baseline';
tests(testCount).pass = timeRatio <= 2.0;
tests(testCount).value = sprintf('RL/Baseline = %.2fx (%.1fs vs %.1fs)', ...
    timeRatio, results_p3.time_rl, results_p3.time_base);

% Check 16: Energy comparison is valid (RL mission used energy)
testCount = testCount+1;
tests(testCount).name = '[P3] RL energy consumption is positive';
tests(testCount).pass = results_p3.energy_rl > 0;
tests(testCount).value = sprintf('%.2f Wh', results_p3.energy_rl/3600);

% Check 17: Reward shows improvement over training
rewardStart = mean(results_p3.episodeRewards(1:50));
rewardEnd   = mean(results_p3.episodeRewards(end-49:end));
testCount = testCount+1;
tests(testCount).name = '[P3] Reward improves over training';
tests(testCount).pass = rewardEnd > rewardStart;
tests(testCount).value = sprintf('Start avg=%.1f -> End avg=%.1f', rewardStart, rewardEnd);

%% -------------------------------------------------------------------------
%  PRINT RESULTS
%% -------------------------------------------------------------------------
fprintf('\n%s\n', repmat('=',1,70));
fprintf('VALIDATION RESULTS\n');
fprintf('%s\n', repmat('=',1,70));

nPass = 0; nFail = 0;
for i = 1:testCount
    if tests(i).pass
        status = 'PASS ✓';
        nPass = nPass + 1;
    else
        status = 'FAIL ✗';
        nFail = nFail + 1;
    end
    fprintf('[%s] %s\n        → %s\n', status, tests(i).name, tests(i).value);
end

fprintf('\n%s\n', repmat('-',1,70));
fprintf('RESULTS: %d/%d tests passed\n', nPass, testCount);
if nFail == 0
    fprintf('✓ ALL VALIDATIONS PASSED - Simulation is VALID\n');
else
    fprintf('✗ %d validation(s) failed - review outputs above\n', nFail);
end
fprintf('%s\n', repmat('=',1,70));

%% -------------------------------------------------------------------------
%  ADDITIONAL DIAGNOSTIC PLOTS
%% -------------------------------------------------------------------------
figure(100);
set(gcf,'Name','Validation Dashboard','Color','white','Position',[50,50,1200,600]);

% P1 voltage profile
subplot(2,3,1);
plot(results_p1.t, results_p1.Vbatt, 'b-', 'LineWidth', 2);
hold on; yline(5.0,'r--','LineWidth',2,'Label','5V limit');
xlabel('Time (s)'); ylabel('V'); title('[P1] Voltage Profile');
ylim([4 18]); grid on;
text(0.5, 0.1, sprintf('Min=%.2fV | %s', minV_p1, ...
    ternary(minV_p1>=5,'PASS ✓','FAIL ✗')), ...
    'Units','normalized','Color',ternary(minV_p1>=5,'g','r'));

% P2 voltage profile
subplot(2,3,2);
plot(results_p2.t, results_p2.Vbatt, 'r-', 'LineWidth', 2);
hold on; yline(5.0,'k--','LineWidth',2,'Label','5V limit');
xlabel('Time (s)'); ylabel('V'); title('[P2] Voltage Profile (Low Batt)');
ylim([4 18]); grid on;
text(0.5, 0.1, sprintf('Min=%.2fV | %s', minV_p2, ...
    ternary(minV_p2>=5,'PASS ✓','FAIL ✗')), ...
    'Units','normalized','Color',ternary(minV_p2>=5,'g','r'));

% Trajectories
subplot(2,3,3);
plot3(results_p1.pos(:,1),results_p1.pos(:,2),results_p1.pos(:,3),'b-','LineWidth',2); hold on;
plot3(results_p2.pos(:,1),results_p2.pos(:,2),results_p2.pos(:,3),'r--','LineWidth',2);
wp = results_p1.waypoints;
scatter3(wp(:,1),wp(:,2),wp(:,3),80,'k','filled');
legend('Normal','Low Batt','Waypoints'); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Trajectory Comparison'); view(45,30);

% RL training reward
subplot(2,3,4);
plot(movmean(results_p3.episodeRewards,20),'b-','LineWidth',2);
xlabel('Episode'); ylabel('Reward'); title('[P3] RL Training Convergence'); grid on;

% RL velocity comparison
subplot(2,3,5);
bar([results_p3.vel_base; results_p3.vel_rl]');
legend('Baseline','RL');
xlabel('Segment'); ylabel('m/s'); title('[P3] Velocity by Segment'); grid on;

% Pass/fail summary
subplot(2,3,6);
axis off;
pct = nPass/testCount*100;
pie([nPass nFail], {sprintf('PASS (%d)',nPass), sprintf('FAIL (%d)',nFail)});
title(sprintf('Validation: %.0f%% pass rate', pct));
colormap([0.2 0.8 0.2; 0.9 0.2 0.2]);

sgtitle('Drone Simulation - Validation Dashboard', 'FontSize', 14, 'FontWeight','bold');

fprintf('\nValidation complete.\n');

%% =========================================================================
function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end
