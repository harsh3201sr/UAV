%% =========================================================================
%  QUADCOPTER DRONE SIMULATION - PART 1: NORMAL OPERATION
%  MATLAB R2024a Compatible
%  6-DOF Quadcopter with PID Control, Battery Model, Waypoint Navigation
%% =========================================================================
clc; clear; close all;

fprintf('===========================================\n');
fprintf(' PART 1: Normal Battery Operation\n');
fprintf('===========================================\n\n');

%% -------------------------------------------------------------------------
%  DRONE PHYSICAL PARAMETERS
%% -------------------------------------------------------------------------
drone.mass      = 1.5;          % kg
drone.g         = 9.81;         % m/s^2
drone.Ixx       = 0.0347;       % kg.m^2 (roll inertia)
drone.Iyy       = 0.0458;       % kg.m^2 (pitch inertia)
drone.Izz       = 0.0977;       % kg.m^2 (yaw inertia)
drone.kT        = 3.13e-5;      % Thrust coefficient (N/(rad/s)^2)
drone.kD        = 7.5e-7;       % Drag coefficient (N.m/(rad/s)^2)
drone.armLen    = 0.225;        % m (arm length)
drone.Cd        = 0.47;         % Aerodynamic drag coefficient
drone.Aref      = 0.04;         % Reference area (m^2)
drone.rhoAir    = 1.225;        % Air density (kg/m^3)
drone.maxThrust = 4 * drone.mass * drone.g * 1.5;  % Max total thrust (N)
drone.maxRPM    = 8000;         % Max rotor RPM

%% -------------------------------------------------------------------------
%  BATTERY PARAMETERS (NORMAL CONDITION)
%% -------------------------------------------------------------------------
batt.capacity   = 5200e-3;      % Ah (5200 mAh)
batt.Vnom       = 14.8;         % Nominal voltage (4S LiPo)
batt.Vfull      = 16.8;         % Fully charged voltage
batt.Vmin       = 13.2;         % Minimum safe voltage
batt.Rint       = 0.05;         % Internal resistance (Ohm)
batt.SoC_init   = 1.0;          % Initial SoC (100%)
batt.Peukert    = 1.05;         % Peukert exponent (nearly ideal)

%% -------------------------------------------------------------------------
%  WAYPOINTS DEFINITION (6 States)
%% -------------------------------------------------------------------------
%  Format: [X, Y, Z] in meters
waypoints = [
    0,   0,   0;    % State 1: Launch (ground)
    10,  0,   5;    % State 2
    10,  10,  8;    % State 3
    20,  10,  10;   % State 4
    20,  20,  12;   % State 5
    30,  20,  5;    % State 6: Landing zone
];
nWaypoints = size(waypoints, 1);
waypointRadius = 0.5;           % Acceptance radius (m)

fprintf('Waypoints defined:\n');
for i = 1:nWaypoints
    fprintf('  State %d: [%.1f, %.1f, %.1f] m\n', i, waypoints(i,1), waypoints(i,2), waypoints(i,3));
end

%% -------------------------------------------------------------------------
%  PID CONTROLLER GAINS
%% -------------------------------------------------------------------------
% Position controllers (outer loop)
PID.pos.Kp = [2.5, 2.5, 4.0];   % [X, Y, Z]
PID.pos.Ki = [0.1, 0.1, 0.3];
PID.pos.Kd = [1.5, 1.5, 2.0];
PID.pos.iMax = [5, 5, 5];        % Integrator saturation

% Attitude controllers (inner loop)
PID.att.Kp = [8.0, 8.0, 4.0];   % [Roll, Pitch, Yaw]
PID.att.Ki = [0.2, 0.2, 0.1];
PID.att.Kd = [2.5, 2.5, 1.5];
PID.att.iMax = [3, 3, 2];

%% -------------------------------------------------------------------------
%  SIMULATION SETTINGS
%% -------------------------------------------------------------------------
dt      = 0.005;        % Time step (s) - 200 Hz
Tmax    = 120;          % Max simulation time (s)
t       = 0:dt:Tmax;
N       = length(t);

% Pre-allocate state arrays
pos     = zeros(N, 3);  % [X, Y, Z] m
vel     = zeros(N, 3);  % [Vx, Vy, Vz] m/s
ang     = zeros(N, 3);  % [Roll, Pitch, Yaw] rad
angRate = zeros(N, 3);  % [p, q, r] rad/s
SoC     = zeros(N, 1);
Vbatt   = zeros(N, 1);
power   = zeros(N, 1);
thrust  = zeros(N, 1);
wpIdx   = zeros(N, 1);  % Current waypoint index

% Initial conditions
pos(1,:)    = waypoints(1,:);
vel(1,:)    = [0, 0, 0];
ang(1,:)    = [0, 0, 0];
angRate(1,:)= [0, 0, 0];
SoC(1)      = batt.SoC_init;
Vbatt(1)    = batt.Vfull;

% PID integrators and previous errors
posErr_int  = zeros(1,3);
attErr_int  = zeros(1,3);
posErr_prev = zeros(1,3);
attErr_prev = zeros(1,3);
currentWP   = 2;        % Start navigating to waypoint 2

fprintf('\nStarting simulation...\n');

%% -------------------------------------------------------------------------
%  MAIN SIMULATION LOOP
%% -------------------------------------------------------------------------
for k = 1:N-1

    wpIdx(k) = currentWP;

    % --- Target waypoint ---
    target = waypoints(currentWP, :);

    % --- Position Error ---
    posErr = target - pos(k,:);

    % --- Check waypoint arrival ---
    if norm(posErr) < waypointRadius && currentWP < nWaypoints
        currentWP = currentWP + 1;
        fprintf('  t=%.1fs | Reached State %d -> heading to State %d\n', ...
                t(k), currentWP-1, currentWP);
        target = waypoints(currentWP, :);
        posErr = target - pos(k,:);
    end

    % =====================================================================
    %  OUTER LOOP: Position PID -> Desired Angles + Altitude Thrust
    % =====================================================================
    posErr_int  = posErr_int  + posErr * dt;
    posErr_int  = max(min(posErr_int, PID.pos.iMax), -PID.pos.iMax);
    posErr_dot  = (posErr - posErr_prev) / dt;
    posErr_prev = posErr;

    % PID output per axis
    posCtrl = PID.pos.Kp .* posErr + ...
              PID.pos.Ki .* posErr_int + ...
              PID.pos.Kd .* posErr_dot;

    % Altitude: directly controls thrust component
    Fz_des = drone.mass * (drone.g + posCtrl(3));
    Fz_des = max(0, min(Fz_des, drone.maxThrust));

    % Horizontal: desired pitch and roll angles
    yaw = ang(k, 3);
    pitch_des = (posCtrl(1)*cos(yaw) + posCtrl(2)*sin(yaw)) / drone.g;
    roll_des  = (posCtrl(1)*sin(yaw) - posCtrl(2)*cos(yaw)) / drone.g;
    pitch_des = max(-0.3, min(pitch_des, 0.3));  % Limit to 17 deg
    roll_des  = max(-0.3, min(roll_des,  0.3));

    % =====================================================================
    %  INNER LOOP: Attitude PID -> Torques
    % =====================================================================
    attTarget = [roll_des, pitch_des, 0];  % Yaw held at 0
    attErr     = attTarget - ang(k,:);
    attErr_int = attErr_int + attErr * dt;
    attErr_int = max(min(attErr_int, PID.att.iMax), -PID.att.iMax);
    attErr_dot = (attErr - attErr_prev) / dt;
    attErr_prev = attErr;

    attCtrl = PID.att.Kp .* attErr + ...
              PID.att.Ki .* attErr_int + ...
              PID.att.Kd .* attErr_dot;

    % Torques
    tau_roll  = attCtrl(1) * drone.Ixx;
    tau_pitch = attCtrl(2) * drone.Iyy;
    tau_yaw   = attCtrl(3) * drone.Izz;

    % =====================================================================
    %  6-DOF DYNAMICS (Euler angles, body frame)
    % =====================================================================
    phi   = ang(k,1);   % Roll
    theta = ang(k,2);   % Pitch
    psi   = ang(k,3);   % Yaw

    % Total thrust
    Ftotal = Fz_des;
    thrust(k) = Ftotal;

    % Rotation matrix (body->world)
    Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
    R  = Rz * Ry * Rx;

    % Force in world frame
    F_world = R * [0; 0; Ftotal];

    % Aerodynamic drag
    v_mag = norm(vel(k,:));
    if v_mag > 0
        F_drag = -0.5 * drone.rhoAir * drone.Cd * drone.Aref * v_mag^2 ...
                  * vel(k,:)/v_mag;
    else
        F_drag = [0, 0, 0];
    end

    % Linear acceleration
    acc = (F_world' + [F_drag(1), F_drag(2), 0] - [0, 0, drone.mass*drone.g]) ...
          / drone.mass;

    % Angular acceleration (simplified Euler equations)
    angAcc(1) = tau_roll  / drone.Ixx;
    angAcc(2) = tau_pitch / drone.Iyy;
    angAcc(3) = tau_yaw   / drone.Izz;

    % Integrate
    vel(k+1,:)     = vel(k,:)     + acc' * dt;
    pos(k+1,:)     = pos(k,:)     + vel(k,:) * dt;
    angRate(k+1,:) = angRate(k,:) + angAcc  * dt;
    ang(k+1,:)     = ang(k,:)     + angRate(k,:) * dt;

    % Ground constraint
    if pos(k+1,3) < 0
        pos(k+1,3) = 0;
        vel(k+1,3) = 0;
    end

    % =====================================================================
    %  BATTERY MODEL (Normal)
    % =====================================================================
    % Power = Thrust * velocity_z + hover_power (simplified)
    hoverPower = (drone.mass * drone.g)^1.5 / sqrt(2 * drone.rhoAir * pi * drone.armLen^2);
    P_mech     = abs(Ftotal) * abs(vel(k,3)) + hoverPower;
    P_elec     = P_mech / 0.85;  % Assume 85% motor efficiency
    power(k)   = P_elec;

    % Current draw
    I_draw = P_elec / max(Vbatt(k), 1);

    % SoC update (Coulomb counting with Peukert)
    dSoC = -(I_draw / (batt.capacity * 3600)) * dt * batt.Peukert;
    SoC(k+1) = max(0, SoC(k) + dSoC);

    % Battery voltage (simple OCV model)
    OCV = batt.Vmin + (batt.Vfull - batt.Vmin) * SoC(k+1);
    Vbatt(k+1) = max(batt.Vmin, OCV - I_draw * batt.Rint);

    % Stop if final waypoint reached with sufficient proximity
    if currentWP == nWaypoints && norm(pos(k+1,:) - waypoints(end,:)) < waypointRadius
        fprintf('  t=%.1fs | MISSION COMPLETE - Reached final State %d!\n', t(k+1), nWaypoints);
        N_actual = k+1;
        break;
    end
    N_actual = N;
end

fprintf('\nSimulation complete. Duration: %.1f s\n', t(N_actual));

%% -------------------------------------------------------------------------
%  SAVE RESULTS
%% -------------------------------------------------------------------------
results_p1.t       = t(1:N_actual);
results_p1.pos     = pos(1:N_actual,:);
results_p1.vel     = vel(1:N_actual,:);
results_p1.ang     = ang(1:N_actual,:);
results_p1.SoC     = SoC(1:N_actual);
results_p1.Vbatt   = Vbatt(1:N_actual);
results_p1.power   = power(1:N_actual);
results_p1.thrust  = thrust(1:N_actual);
results_p1.wpIdx   = wpIdx(1:N_actual);
results_p1.waypoints = waypoints;
results_p1.N_actual  = N_actual;

save('results_part1.mat', 'results_p1', 'drone', 'batt');

%% -------------------------------------------------------------------------
%  PRINT SUMMARY
%% -------------------------------------------------------------------------
fprintf('\n--- PART 1 SUMMARY ---\n');
fprintf('  Final SoC          : %.1f%%\n',  SoC(N_actual)*100);
fprintf('  Final Voltage      : %.2f V\n', Vbatt(N_actual));
fprintf('  Min Voltage        : %.2f V\n', min(Vbatt(1:N_actual)));
fprintf('  Avg Power          : %.1f W\n', mean(power(1:N_actual)));
fprintf('  Total Energy Used  : %.2f Wh\n', sum(power(1:N_actual))*dt/3600);
fprintf('  Max Speed          : %.2f m/s\n', max(vecnorm(vel(1:N_actual,:),2,2)));
fprintf('  Voltage Constraint : %s (min 5V)\n', ...
        ternary(min(Vbatt(1:N_actual))>=5, 'PASS ✓', 'FAIL ✗'));

%% -------------------------------------------------------------------------
%  PLOTS
%% -------------------------------------------------------------------------
plot_drone_results(results_p1, 'Part 1: Normal Operation', 1);

fprintf('\nPart 1 complete. Results saved to results_part1.mat\n');

%% =========================================================================
%  HELPER FUNCTIONS
%% =========================================================================
function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end
