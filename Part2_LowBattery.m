%% =========================================================================
%  QUADCOPTER DRONE SIMULATION - PART 2: LOW BATTERY / POWER MANAGEMENT
%  MATLAB R2024a Compatible
%  Degraded battery model, voltage constraints, power-aware flight
%% =========================================================================
clc; clear; close all;

fprintf('===========================================\n');
fprintf(' PART 2: Low Battery / Power Management\n');
fprintf('===========================================\n\n');

%% -------------------------------------------------------------------------
%  DRONE PARAMETERS (identical to Part 1)
%% -------------------------------------------------------------------------
drone.mass      = 1.5;
drone.g         = 9.81;
drone.Ixx       = 0.0347;
drone.Iyy       = 0.0458;
drone.Izz       = 0.0977;
drone.kT        = 3.13e-5;
drone.kD        = 7.5e-7;
drone.armLen    = 0.225;
drone.Cd        = 0.47;
drone.Aref      = 0.04;
drone.rhoAir    = 1.225;
drone.maxThrust = 4 * drone.mass * drone.g * 1.5;

%% -------------------------------------------------------------------------
%  DEGRADED BATTERY MODEL ("Fake Model Intake Power Drop")
%% -------------------------------------------------------------------------
batt.capacity      = 5200e-3;     % Ah
batt.Vnom          = 14.8;        % Nominal voltage
batt.Vfull         = 15.5;        % Reduced because already partially used
batt.Vmin          = 5.0;         % HARD minimum - safety cutoff
batt.Vpref         = 6.0;         % Preferred minimum (electronics safe)
batt.Rint          = 0.18;        % INCREASED internal resistance (degraded)
batt.SoC_init      = 0.40;        % Start at 40% SoC (low battery)
batt.Peukert       = 1.25;        % Higher Peukert - faster real discharge
batt.degFactor     = 2.8;         % Artificial degradation multiplier
batt.selfDischarge = 0.0002;      % Self-discharge rate per second

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
nWaypoints       = size(waypoints, 1);
waypointRadius   = 0.8;     % Slightly larger acceptance radius

%% -------------------------------------------------------------------------
%  PID GAINS (same as Part 1)
%% -------------------------------------------------------------------------
PID.pos.Kp = [2.5, 2.5, 4.0];
PID.pos.Ki = [0.1, 0.1, 0.3];
PID.pos.Kd = [1.5, 1.5, 2.0];
PID.pos.iMax = [5, 5, 5];

PID.att.Kp = [8.0, 8.0, 4.0];
PID.att.Ki = [0.2, 0.2, 0.1];
PID.att.Kd = [2.5, 2.5, 1.5];
PID.att.iMax = [3, 3, 2];

%% -------------------------------------------------------------------------
%  POWER MANAGEMENT THRESHOLDS
%% -------------------------------------------------------------------------
pm.SoC_warn      = 0.25;    % 25% SoC - start throttling
pm.SoC_critical  = 0.15;    % 15% SoC - aggressive throttle
pm.Vwarn         = 10.0;    % Voltage warning threshold
pm.Vcritical     = 7.0;     % Critical voltage - emergency slow
pm.speedFactor_warn     = 0.7;   % Reduce speed to 70% at warning
pm.speedFactor_critical = 0.45;  % Reduce speed to 45% at critical
pm.maxAngle_normal  = 0.30;  % rad (~17 deg)
pm.maxAngle_low     = 0.15;  % rad (~9 deg) - conservative at low batt

%% -------------------------------------------------------------------------
%  SIMULATION SETTINGS
%% -------------------------------------------------------------------------
dt      = 0.005;
Tmax    = 200;          % Extended time for low-battery scenario
t       = 0:dt:Tmax;
N       = length(t);

% Pre-allocate
pos     = zeros(N, 3);
vel     = zeros(N, 3);
ang     = zeros(N, 3);
angRate = zeros(N, 3);
SoC     = zeros(N, 1);
Vbatt   = zeros(N, 1);
power   = zeros(N, 1);
thrust  = zeros(N, 1);
wpIdx   = zeros(N, 1);
speedFactor = ones(N, 1);
safetyEvents = struct('time', {}, 'type', {}, 'voltage', {}, 'SoC', {});

% Initial conditions
pos(1,:)    = waypoints(1,:);
vel(1,:)    = [0, 0, 0];
ang(1,:)    = [0, 0, 0];
angRate(1,:)= [0, 0, 0];
SoC(1)      = batt.SoC_init;
Vbatt(1)    = batt.Vmin + (batt.Vfull - batt.Vmin) * batt.SoC_init;

posErr_int  = zeros(1,3);
attErr_int  = zeros(1,3);
posErr_prev = zeros(1,3);
attErr_prev = zeros(1,3);
currentWP   = 2;
N_actual    = N;
safetyActive = false;

fprintf('Starting LOW BATTERY simulation...\n');
fprintf('Initial SoC: %.1f%% | Initial Voltage: %.2fV\n', ...
        SoC(1)*100, Vbatt(1));
fprintf('WARNING: Voltage constraints: Hard min=%.1fV, Preferred=%.1fV\n\n', ...
        batt.Vmin, batt.Vpref);

%% -------------------------------------------------------------------------
%  MAIN SIMULATION LOOP
%% -------------------------------------------------------------------------
for k = 1:N-1

    wpIdx(k) = currentWP;

    % --- Power Management: Compute speed scaling factor ---
    sf = 1.0;  % Default: full speed
    if Vbatt(k) < pm.Vcritical || SoC(k) < pm.SoC_critical
        sf = pm.speedFactor_critical;
        if ~safetyActive
            ev.time = t(k); ev.type = 'CRITICAL';
            ev.voltage = Vbatt(k); ev.SoC = SoC(k);
            safetyEvents(end+1) = ev;
            fprintf('  t=%.1fs | *** CRITICAL POWER *** V=%.2fV SoC=%.1f%% -> Speed %.0f%%\n', ...
                    t(k), Vbatt(k), SoC(k)*100, sf*100);
            safetyActive = true;
        end
    elseif Vbatt(k) < pm.Vwarn || SoC(k) < pm.SoC_warn
        sf = pm.speedFactor_warn;
        if ~safetyActive
            ev.time = t(k); ev.type = 'WARNING';
            ev.voltage = Vbatt(k); ev.SoC = SoC(k);
            safetyEvents(end+1) = ev;
            fprintf('  t=%.1fs | * Power Warning * V=%.2fV SoC=%.1f%% -> Speed %.0f%%\n', ...
                    t(k), Vbatt(k), SoC(k)*100, sf*100);
            safetyActive = true;
        end
    else
        safetyActive = false;
    end
    speedFactor(k) = sf;

    % Angle limits based on power state
    if sf < pm.speedFactor_warn
        maxAngle = pm.maxAngle_low;
    else
        maxAngle = pm.maxAngle_normal;
    end

    % --- Target waypoint ---
    target = waypoints(currentWP, :);
    posErr = target - pos(k,:);

    if norm(posErr) < waypointRadius && currentWP < nWaypoints
        currentWP = currentWP + 1;
        fprintf('  t=%.1fs | Reached State %d -> heading to State %d | V=%.2fV SoC=%.1f%%\n', ...
                t(k), currentWP-1, currentWP, Vbatt(k), SoC(k)*100);
        target = waypoints(currentWP, :);
        posErr = target - pos(k,:);
    end

    % =====================================================================
    %  PID POSITION CONTROL (with speed scaling)
    % =====================================================================
    posErr_int  = posErr_int  + posErr * dt;
    posErr_int  = max(min(posErr_int, PID.pos.iMax), -PID.pos.iMax);
    posErr_dot  = (posErr - posErr_prev) / dt;
    posErr_prev = posErr;

    posCtrl = sf * (PID.pos.Kp .* posErr + ...
                    PID.pos.Ki .* posErr_int + ...
                    PID.pos.Kd .* posErr_dot);

    Fz_des = drone.mass * (drone.g + posCtrl(3));
    Fz_des = max(0, min(Fz_des, drone.maxThrust));

    yaw = ang(k, 3);
    pitch_des = (posCtrl(1)*cos(yaw) + posCtrl(2)*sin(yaw)) / drone.g;
    roll_des  = (posCtrl(1)*sin(yaw) - posCtrl(2)*cos(yaw)) / drone.g;

    % Apply conservative angle limits at low battery
    pitch_des = max(-maxAngle, min(pitch_des, maxAngle));
    roll_des  = max(-maxAngle, min(roll_des,  maxAngle));

    % =====================================================================
    %  PID ATTITUDE CONTROL
    % =====================================================================
    attTarget   = [roll_des, pitch_des, 0];
    attErr      = attTarget - ang(k,:);
    attErr_int  = attErr_int + attErr * dt;
    attErr_int  = max(min(attErr_int, PID.att.iMax), -PID.att.iMax);
    attErr_dot  = (attErr - attErr_prev) / dt;
    attErr_prev = attErr;

    attCtrl = PID.att.Kp .* attErr + ...
              PID.att.Ki .* attErr_int + ...
              PID.att.Kd .* attErr_dot;

    tau_roll  = attCtrl(1) * drone.Ixx;
    tau_pitch = attCtrl(2) * drone.Iyy;
    tau_yaw   = attCtrl(3) * drone.Izz;

    % =====================================================================
    %  6-DOF DYNAMICS
    % =====================================================================
    phi = ang(k,1); theta = ang(k,2); psi = ang(k,3);
    Ftotal = Fz_des;
    thrust(k) = Ftotal;

    Rz = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
    Ry = [cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];
    Rx = [1,0,0; 0,cos(phi),-sin(phi); 0,sin(phi),cos(phi)];
    R  = Rz*Ry*Rx;

    F_world = R * [0; 0; Ftotal];
    v_mag   = norm(vel(k,:));
    if v_mag > 0
        F_drag = -0.5 * drone.rhoAir * drone.Cd * drone.Aref * v_mag^2 ...
                  * vel(k,:)/v_mag;
    else
        F_drag = [0,0,0];
    end

    acc    = (F_world' + [F_drag(1),F_drag(2),0] - [0,0,drone.mass*drone.g]) ...
             / drone.mass;
    angAcc = [tau_roll/drone.Ixx, tau_pitch/drone.Iyy, tau_yaw/drone.Izz];

    vel(k+1,:)     = vel(k,:)     + acc'   * dt;
    pos(k+1,:)     = pos(k,:)     + vel(k,:) * dt;
    angRate(k+1,:) = angRate(k,:) + angAcc * dt;
    ang(k+1,:)     = ang(k,:)     + angRate(k,:) * dt;

    if pos(k+1,3) < 0; pos(k+1,3)=0; vel(k+1,3)=0; end

    % =====================================================================
    %  DEGRADED BATTERY MODEL
    % =====================================================================
    hoverPower = (drone.mass * drone.g)^1.5 / sqrt(2*drone.rhoAir*pi*drone.armLen^2);
    P_mech     = abs(Ftotal) * abs(vel(k,3)) + hoverPower;
    P_elec     = P_mech / 0.85;

    % Apply degradation factor (simulates faster-than-expected discharge)
    P_effective = P_elec * batt.degFactor;
    power(k)    = P_effective;

    I_draw = P_effective / max(Vbatt(k), 1);

    % Peukert effect: effective capacity reduces with high current
    I_rated = batt.capacity;  % 1C rate
    I_peuk  = I_draw / I_rated;
    dSoC_base = -(I_draw / (batt.capacity * 3600)) * dt;
    dSoC = dSoC_base * batt.Peukert * max(1, I_peuk^0.3);

    % Self-discharge
    dSoC = dSoC - batt.selfDischarge * SoC(k) * dt;
    SoC(k+1) = max(0, SoC(k) + dSoC);

    % Open-circuit voltage (includes SoC-dependent drop)
    OCV_map_SoC = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
    OCV_map_V   = [5.0, 5.8, 7.2, 9.0, 11.0, 12.5, 13.5, 14.2, 14.8, 15.4, 15.5];
    OCV = interp1(OCV_map_SoC, OCV_map_V, SoC(k+1), 'pchip', 'extrap');
    Vbatt(k+1) = max(batt.Vmin, OCV - I_draw * batt.Rint);

    % *** HARD VOLTAGE CONSTRAINT ***
    if Vbatt(k+1) < batt.Vmin
        Vbatt(k+1) = batt.Vmin;
        fprintf('  t=%.1fs | !!! VOLTAGE CLAMPED to %.1fV (hard limit) !!!\n', ...
                t(k+1), batt.Vmin);
    end

    % Stop if final waypoint reached
    if currentWP == nWaypoints && norm(pos(k+1,:) - waypoints(end,:)) < waypointRadius
        fprintf('\n  t=%.1fs | MISSION COMPLETE - Reached final State %d!\n', t(k+1), nWaypoints);
        N_actual = k+1;
        break;
    end

    % Emergency: if battery truly dead, forced landing
    if SoC(k+1) <= 0
        fprintf('\n  t=%.1fs | !!! BATTERY DEPLETED - EMERGENCY LANDING !!!\n', t(k+1));
        N_actual = k+1;
        break;
    end
end

%% -------------------------------------------------------------------------
%  SAVE RESULTS
%% -------------------------------------------------------------------------
results_p2.t            = t(1:N_actual);
results_p2.pos          = pos(1:N_actual,:);
results_p2.vel          = vel(1:N_actual,:);
results_p2.ang          = ang(1:N_actual,:);
results_p2.SoC          = SoC(1:N_actual);
results_p2.Vbatt        = Vbatt(1:N_actual);
results_p2.power        = power(1:N_actual);
results_p2.speedFactor  = speedFactor(1:N_actual);
results_p2.safetyEvents = safetyEvents;
results_p2.waypoints    = waypoints;
results_p2.N_actual     = N_actual;

save('results_part2.mat', 'results_p2', 'drone', 'batt', 'pm');

%% -------------------------------------------------------------------------
%  PRINT SUMMARY
%% -------------------------------------------------------------------------
minV = min(Vbatt(1:N_actual));
fprintf('\n--- PART 2 SUMMARY ---\n');
fprintf('  Mission Duration       : %.1f s\n', t(N_actual));
fprintf('  Final SoC              : %.1f%%\n', SoC(N_actual)*100);
fprintf('  Min Battery Voltage    : %.2f V\n', minV);
fprintf('  Hard Voltage Limit     : 5.0 V\n');
fprintf('  Voltage Constraint     : %s\n', ternary(minV >= 5.0, 'PASS ✓ (>=5V)', 'FAIL ✗'));
fprintf('  Preferred V Maintained : %s\n', ternary(minV >= 6.0, 'YES (>=6V)', 'NO (<6V at times)'));
fprintf('  Safety Events Logged   : %d\n', length(safetyEvents));
fprintf('  Total Energy Used      : %.2f Wh\n', sum(power(1:N_actual))*dt/3600);
for i = 1:length(safetyEvents)
    fprintf('    [%s] at t=%.1fs V=%.2fV SoC=%.1f%%\n', ...
            safetyEvents(i).type, safetyEvents(i).time, ...
            safetyEvents(i).voltage, safetyEvents(i).SoC*100);
end

%% -------------------------------------------------------------------------
%  PLOTS
%% -------------------------------------------------------------------------
plot_drone_results(results_p2, 'Part 2: Low Battery Operation', 2);

% Additional Part 2 specific: speed factor over time
figure(20);
set(gcf,'Name','Part 2: Power Management Interventions','Color','white');
subplot(2,1,1);
plot(results_p2.t, results_p2.Vbatt, 'r-', 'LineWidth', 2); hold on;
yline(batt.Vmin,  'k--', 'LineWidth', 2, 'Label', 'Hard Min 5V');
yline(batt.Vpref, 'b--', 'LineWidth', 1.5, 'Label', 'Preferred 6V');
xlabel('Time (s)'); ylabel('Voltage (V)');
title('Battery Voltage vs Time (Part 2)');
legend('Vbatt','Hard Min 5V','Preferred 6V','Location','northeast');
grid on;

subplot(2,1,2);
plot(results_p2.t, results_p2.speedFactor*100, 'm-', 'LineWidth', 2);
ylim([0 120]); grid on;
xlabel('Time (s)'); ylabel('Speed Factor (%)');
title('Speed Reduction Factor from Power Management');
yline(100,'k--','Normal'); yline(70,'b--','Warning Level'); yline(45,'r--','Critical Level');

fprintf('\nPart 2 complete. Results saved to results_part2.mat\n');

%% =========================================================================
function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end
