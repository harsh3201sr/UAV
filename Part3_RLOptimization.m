%% =========================================================================
%  QUADCOPTER DRONE SIMULATION - PART 3: RL VELOCITY OPTIMIZATION
%  MATLAB R2024a Compatible
%  Q-Learning agent for optimal velocity profile across waypoints
%% =========================================================================
clc; clear; close all;

fprintf('===========================================\n');
fprintf(' PART 3: Reinforcement Learning (Q-Learning)\n');
fprintf('         Velocity Optimization\n');
fprintf('===========================================\n\n');

%% -------------------------------------------------------------------------
%  DRONE & BATTERY PARAMETERS
%% -------------------------------------------------------------------------
drone.mass   = 1.5;
drone.g      = 9.81;
drone.armLen = 0.225;
drone.rhoAir = 1.225;

batt.capacity = 5200e-3;
batt.Vfull    = 16.8;
batt.Vmin     = 5.0;
batt.Rint     = 0.08;
batt.SoC_init = 0.85;  % 85% start

%% -------------------------------------------------------------------------
%  WAYPOINTS
%% -------------------------------------------------------------------------
waypoints = [
    0,   0,   0;
    10,  0,   5;
    10,  10,  8;
    20,  10,  10;
    20,  20,  12;
    30,  20,  5;
];
nWaypoints = size(waypoints, 1);

% Segment distances
segDist = zeros(nWaypoints-1, 1);
for i = 1:nWaypoints-1
    segDist(i) = norm(waypoints(i+1,:) - waypoints(i,:));
end
totalDist = sum(segDist);
fprintf('Total path distance: %.1f m\n\n', totalDist);

%% -------------------------------------------------------------------------
%  RL ENVIRONMENT DEFINITION
%% -------------------------------------------------------------------------
% State space:
%   s1 = Waypoint index  (1..5, leg being flown)
%   s2 = Velocity bin    (1..5: very slow to fast)
%   s3 = SoC bin         (1..5: critical to full)
%
% Action space:
%   1 = Decrease velocity (-1 m/s)
%   2 = Maintain velocity
%   3 = Increase velocity (+1 m/s)

nWPStates  = nWaypoints - 1;   % 5 legs
nVelBins   = 5;                 % velocity levels
nSoCBins   = 5;                 % SoC levels

nStates  = nWPStates * nVelBins * nSoCBins;
nActions = 3;

% Velocity levels (m/s)
velLevels = [1.0, 2.5, 4.0, 5.5, 7.0];   % Bins 1-5

% SoC bin edges
SoC_edges = [0, 0.15, 0.30, 0.55, 0.75, 1.01];

%% -------------------------------------------------------------------------
%  Q-LEARNING HYPERPARAMETERS
%% -------------------------------------------------------------------------
RL.alpha     = 0.15;     % Learning rate
RL.gamma     = 0.90;     % Discount factor
RL.epsilon0  = 1.0;      % Initial exploration rate
RL.epsilon_min = 0.05;   % Minimum exploration
RL.epsilon_decay = 0.995;% Per-episode decay
RL.nEpisodes = 600;      % Training episodes
RL.maxSteps  = 200;      % Max steps per episode

% Reward weights
RW.time     = -0.5;     % Penalty per second
RW.energy   = -0.3;     % Penalty per Joule
RW.arrival  = +200;     % Reward for reaching final waypoint
RW.waypoint = +30;      % Reward per intermediate waypoint
RW.lowBatt  = -80;      % Penalty: SoC < 15%
RW.critBatt = -200;     % Penalty: Voltage < 6V
RW.fastFin  = +100;     % Bonus: complete with SoC > 50%

%% -------------------------------------------------------------------------
%  INITIALIZE Q-TABLE
%% -------------------------------------------------------------------------
Q = zeros(nStates, nActions);

% State encoding helper
encode = @(wp, vb, sb) (wp-1)*nVelBins*nSoCBins + (vb-1)*nSoCBins + sb;

% Bin helpers
getBin = @(val, edges) max(1, min(length(edges)-1, ...
    find(val >= edges(1:end-1) & val < edges(2:end), 1, 'first')));
getVelBin = @(v) max(1, min(nVelBins, ...
    round(interp1(velLevels, 1:nVelBins, v, 'linear', 'extrap'))));

%% -------------------------------------------------------------------------
%  TRAINING LOOP
%% -------------------------------------------------------------------------
fprintf('Training Q-Learning agent...\n');
fprintf('Episodes: %d | States: %d | Actions: %d\n\n', ...
        RL.nEpisodes, nStates, nActions);

episodeRewards   = zeros(RL.nEpisodes, 1);
episodeTimes     = zeros(RL.nEpisodes, 1);
episodeEnergy    = zeros(RL.nEpisodes, 1);
episodeSuccess   = zeros(RL.nEpisodes, 1);
epsilon          = RL.epsilon0;

hoverPower = (drone.mass * drone.g)^1.5 / ...
              sqrt(2 * drone.rhoAir * pi * drone.armLen^2);

for ep = 1:RL.nEpisodes

    % --- Reset environment ---
    SoC_sim  = batt.SoC_init;
    Vbatt_sim = batt.Vfull * SoC_sim;
    vel_sim  = velLevels(3);    % Start at mid-speed (4 m/s)
    wp_sim   = 1;               % Current leg (1 = leg from wp1 to wp2)
    totalTime    = 0;
    totalEnergy  = 0;
    totalReward  = 0;
    done = false;

    % Initial state
    vb = getVelBin(vel_sim);
    sb = getBin(SoC_sim, SoC_edges);
    state = encode(wp_sim, vb, sb);

    for step = 1:RL.maxSteps

        % --- Epsilon-greedy action selection ---
        if rand() < epsilon
            action = randi(nActions);
        else
            [~, action] = max(Q(state, :));
        end

        % --- Apply action ---
        switch action
            case 1  % Decrease velocity
                vel_new = max(velLevels(1), vel_sim - 1.5);
            case 2  % Maintain
                vel_new = vel_sim;
            case 3  % Increase velocity
                vel_new = min(velLevels(end), vel_sim + 1.5);
        end

        % --- Simulate leg traversal ---
        d_leg    = segDist(wp_sim);
        t_leg    = d_leg / vel_new;          % Time for this leg

        % Power model: hover + motion power
        P_motion = 0.5 * drone.rhoAir * drone.Cd * 0.04 * vel_new^3;
        P_total  = (hoverPower + P_motion) / 0.85;
        E_leg    = P_total * t_leg;

        % SoC update
        I_draw   = P_total / max(Vbatt_sim, 1);
        dSoC     = -(I_draw / (batt.capacity * 3600)) * t_leg * 1.05;
        SoC_new  = max(0, SoC_sim + dSoC);
        OCV_new  = batt.Vmin + (batt.Vfull - batt.Vmin) * SoC_new;
        Vbatt_new = max(batt.Vmin, OCV_new - I_draw * batt.Rint);

        totalTime   = totalTime + t_leg;
        totalEnergy = totalEnergy + E_leg;

        % --- Compute reward ---
        r = RW.time * t_leg + RW.energy * E_leg;

        % Waypoint reached
        if wp_sim < nWPStates
            r = r + RW.waypoint;
            wp_new = wp_sim + 1;
        else
            % Final destination
            r = r + RW.arrival;
            if SoC_new > 0.50
                r = r + RW.fastFin;
            end
            done = true;
            episodeSuccess(ep) = 1;
        end

        % Safety penalties
        if SoC_new < 0.15
            r = r + RW.lowBatt;
        end
        if Vbatt_new < 6.0
            r = r + RW.critBatt;
        end

        totalReward = totalReward + r;

        % --- Next state ---
        vb_new = getVelBin(vel_new);
        sb_new = getBin(SoC_new, SoC_edges);
        if done
            next_state = state;  % Terminal
        else
            next_state = encode(wp_new, vb_new, sb_new);
        end

        % --- Q-Update (Bellman equation) ---
        if done
            td_target = r;
        else
            td_target = r + RL.gamma * max(Q(next_state, :));
        end
        Q(state, action) = Q(state, action) + ...
                           RL.alpha * (td_target - Q(state, action));

        % --- Transition ---
        state     = next_state;
        SoC_sim   = SoC_new;
        Vbatt_sim = Vbatt_new;
        vel_sim   = vel_new;
        if wp_sim < nWPStates; wp_sim = wp_new; end

        if done; break; end
    end

    episodeRewards(ep)  = totalReward;
    episodeTimes(ep)    = totalTime;
    episodeEnergy(ep)   = totalEnergy;

    % Decay epsilon
    epsilon = max(RL.epsilon_min, epsilon * RL.epsilon_decay);

    % Progress reporting
    if mod(ep, 100) == 0
        successRate = mean(episodeSuccess(max(1,ep-99):ep)) * 100;
        fprintf('  Episode %3d/%d | Avg Reward: %6.1f | Success: %.0f%% | epsilon=%.3f\n', ...
                ep, RL.nEpisodes, mean(episodeRewards(max(1,ep-99):ep)), ...
                successRate, epsilon);
    end
end

fprintf('\nTraining complete!\n');
fprintf('Final success rate: %.1f%%\n', mean(episodeSuccess(end-99:end))*100);

%% -------------------------------------------------------------------------
%  EXTRACT OPTIMAL POLICY
%% -------------------------------------------------------------------------
fprintf('\nExtracting optimal velocity policy...\n');
optPolicy = zeros(nWPStates, nVelBins, nSoCBins);
for wp = 1:nWPStates
    for vb = 1:nVelBins
        for sb = 1:nSoCBins
            s = encode(wp, vb, sb);
            [~, a] = max(Q(s,:));
            optPolicy(wp, vb, sb) = a;
        end
    end
end

%% -------------------------------------------------------------------------
%  RUN: BASELINE (No RL - fixed mid velocity)
%% -------------------------------------------------------------------------
fprintf('\nRunning BASELINE simulation (fixed velocity, no RL)...\n');
[time_base, energy_base, SoC_base, vel_base] = ...
    simulate_mission(waypoints, segDist, velLevels(3), ...  % Fixed 4 m/s
                     batt, drone, hoverPower, false, Q, getVelBin, getBin, SoC_edges, nSoCBins);

%% -------------------------------------------------------------------------
%  RUN: WITH RL POLICY
%% -------------------------------------------------------------------------
fprintf('Running RL-OPTIMIZED simulation...\n');
[time_rl, energy_rl, SoC_rl, vel_rl] = ...
    simulate_mission(waypoints, segDist, velLevels(3), ...
                     batt, drone, hoverPower, true, Q, getVelBin, getBin, SoC_edges, nSoCBins);

%% -------------------------------------------------------------------------
%  SAVE AND REPORT
%% -------------------------------------------------------------------------
results_p3.Q               = Q;
results_p3.optPolicy       = optPolicy;
results_p3.episodeRewards  = episodeRewards;
results_p3.episodeTimes    = episodeTimes;
results_p3.episodeEnergy   = episodeEnergy;
results_p3.episodeSuccess  = episodeSuccess;
results_p3.time_base       = time_base;
results_p3.energy_base     = energy_base;
results_p3.SoC_base        = SoC_base;
results_p3.vel_base        = vel_base;
results_p3.time_rl         = time_rl;
results_p3.energy_rl       = energy_rl;
results_p3.SoC_rl          = SoC_rl;
results_p3.vel_rl          = vel_rl;
results_p3.waypoints       = waypoints;
results_p3.velLevels       = velLevels;
results_p3.RL              = RL;

save('results_part3.mat', 'results_p3');

fprintf('\n--- PART 3 COMPARISON ---\n');
fprintf('                        Baseline     RL-Optimized   Change\n');
fprintf('  Mission Time (s)   : %8.1f     %8.1f    %+.1f%%\n', ...
        time_base, time_rl, (time_rl-time_base)/time_base*100);
fprintf('  Energy Used (Wh)   : %8.2f     %8.2f    %+.1f%%\n', ...
        energy_base/3600, energy_rl/3600, (energy_rl-energy_base)/energy_base*100);
fprintf('  Final SoC (%%)      : %8.1f     %8.1f    %+.1f pp\n', ...
        SoC_base(end)*100, SoC_rl(end)*100, (SoC_rl(end)-SoC_base(end))*100);
fprintf('  Episodes Trained   : %d\n', RL.nEpisodes);

%% -------------------------------------------------------------------------
%  PLOTS - PART 3
%% -------------------------------------------------------------------------
% Training convergence
figure(30);
set(gcf,'Name','Part 3: RL Training','Color','white','Position',[100,100,1200,800]);

subplot(2,2,1);
window = 20;
smoothR = movmean(episodeRewards, window);
plot(episodeRewards, 'Color',[0.8 0.8 0.8]); hold on;
plot(smoothR, 'b-', 'LineWidth', 2);
xlabel('Episode'); ylabel('Total Reward');
title('Training Reward Convergence');
legend('Per Episode','Moving Avg','Location','southeast');
grid on;

subplot(2,2,2);
smoothS = movmean(episodeSuccess, window)*100;
plot(smoothS, 'g-', 'LineWidth', 2);
xlabel('Episode'); ylabel('Success Rate (%)');
title('Mission Success Rate During Training');
ylim([0 110]); grid on;

subplot(2,2,3);
smoothT = movmean(episodeTimes, window);
plot(smoothT, 'r-', 'LineWidth', 2);
xlabel('Episode'); ylabel('Mission Time (s)');
title('Mission Time During Training');
grid on;

subplot(2,2,4);
smoothE = movmean(episodeEnergy/3600, window);
plot(smoothE, 'm-', 'LineWidth', 2);
xlabel('Episode'); ylabel('Energy (Wh)');
title('Energy Consumption During Training');
grid on;

% Comparison plot
figure(31);
set(gcf,'Name','Part 3: RL vs Baseline Comparison','Color','white','Position',[100,100,1200,600]);
legs = 1:nWaypoints-1;

subplot(1,3,1);
bar(legs, [vel_base, vel_rl]);
xlabel('Path Segment'); ylabel('Velocity (m/s)');
title('Velocity Profile: Baseline vs RL');
legend('Baseline','RL Optimized');
xticklabels({'WP1→2','WP2→3','WP3→4','WP4→5','WP5→6'});
xtickangle(30); grid on;

subplot(1,3,2);
bar([time_base, time_rl]);
set(gca,'XTickLabel',{'Baseline','RL Optimized'});
ylabel('Time (s)'); title('Total Mission Time');
grid on; colororder({'#D95319','#0072BD'});

subplot(1,3,3);
bar([energy_base/3600, energy_rl/3600]);
set(gca,'XTickLabel',{'Baseline','RL Optimized'});
ylabel('Energy (Wh)'); title('Total Energy Consumed');
grid on;

fprintf('\nPart 3 complete. Results saved to results_part3.mat\n');

%% =========================================================================
%  MISSION SIMULATION FUNCTION
%% =========================================================================
function [totalTime, totalEnergy, SoC_trace, vel_trace] = ...
    simulate_mission(waypoints, segDist, initVel, batt, drone, hoverPower, ...
                     useRL, Q, getVelBin, getBin, SoC_edges, nSoCBins)

    velLevels = [1.0, 2.5, 4.0, 5.5, 7.0];
    nWPStates = size(waypoints,1) - 1;
    nVelBins  = 5;

    encode = @(wp, vb, sb) (wp-1)*nVelBins*nSoCBins + (vb-1)*nSoCBins + sb;
    getVBin = @(v) max(1, min(nVelBins, ...
        round(interp1(velLevels, 1:nVelBins, v, 'linear', 'extrap'))));

    SoC_sim   = batt.SoC_init;
    Vbatt_sim = batt.Vfull * SoC_sim;
    vel_sim   = initVel;
    totalTime = 0; totalEnergy = 0;
    SoC_trace = zeros(nWPStates, 1);
    vel_trace = zeros(nWPStates, 1);

    for leg = 1:nWPStates
        if useRL
            vb = getVBin(vel_sim);
            sb = getBin(SoC_sim, SoC_edges);
            s  = encode(leg, vb, sb);
            [~, a] = max(Q(s,:));
            switch a
                case 1; vel_sim = max(velLevels(1), vel_sim - 1.5);
                case 3; vel_sim = min(velLevels(end), vel_sim + 1.5);
            end
        end

        d_leg   = segDist(leg);
        t_leg   = d_leg / vel_sim;
        P_motion = 0.5 * drone.rhoAir * 0.47 * 0.04 * vel_sim^3;
        P_total  = (hoverPower + P_motion) / 0.85;
        E_leg    = P_total * t_leg;
        I_draw   = P_total / max(Vbatt_sim, 1);
        dSoC     = -(I_draw / (batt.capacity * 3600)) * t_leg * 1.05;
        SoC_sim  = max(0, SoC_sim + dSoC);
        OCV      = batt.Vmin + (batt.Vfull - batt.Vmin) * SoC_sim;
        Vbatt_sim = max(batt.Vmin, OCV - I_draw * batt.Rint);

        totalTime   = totalTime + t_leg;
        totalEnergy = totalEnergy + E_leg;
        SoC_trace(leg) = SoC_sim;
        vel_trace(leg) = vel_sim;
    end
end
