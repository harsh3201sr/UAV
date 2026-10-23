%% =========================================================================
%  README - Quadcopter Drone Simulation (MATLAB R2024a)
%  =========================================================================
%
%  PROJECT STRUCTURE
%  -----------------
%  RunAll_DroneSimulation.m     <- MASTER SCRIPT (start here)
%  Part1_NormalOperation.m      <- Part 1: 6-DOF drone, normal battery
%  Part2_LowBattery.m           <- Part 2: Degraded battery, power management
%  Part3_RLOptimization.m       <- Part 3: Q-learning velocity optimizer
%  Create_Simulink_Model.m      <- Generates .slx block diagram
%  plot_drone_results.m         <- Shared plotting function (auto-called)
%  Validate_DroneSimulation.m   <- Post-run validation suite
%
%  HOW TO RUN
%  ----------
%  1. Open MATLAB R2024a
%  2. Set current folder to this directory
%  3. Run: RunAll_DroneSimulation        (runs all 3 parts + comparison)
%     Or individually:
%       Part1_NormalOperation            (generates results_part1.mat)
%       Part2_LowBattery                 (generates results_part2.mat)
%       Part3_RLOptimization             (generates results_part3.mat)
%  4. After running, validate: Validate_DroneSimulation
%  5. Create Simulink model: Create_Simulink_Model
%
%  OUTPUTS GENERATED
%  -----------------
%  results_part1.mat            <- Part 1 simulation data
%  results_part2.mat            <- Part 2 simulation data
%  results_part3.mat            <- Part 3 RL training data
%  QuadcopterDroneSim.slx       <- Simulink block diagram
%
%  PLOTS (auto-generated per part)
%  --------------------------------
%  Fig X1: 3D trajectory with waypoints
%  Fig X2: X/Y/Z position vs time
%  Fig X3: Velocity components (Vx,Vy,Vz,|V|)
%  Fig X4: Battery SoC, voltage, power
%  Fig X5: Summary dashboard
%  Fig 99: Cross-part comparison
%  Fig 100: Validation dashboard (after Validate_DroneSimulation.m)
%
%  DRONE PARAMETERS
%  ----------------
%  Mass            : 1.5 kg
%  Inertia (Ixx)   : 0.0347 kg.m^2
%  Inertia (Iyy)   : 0.0458 kg.m^2
%  Inertia (Izz)   : 0.0977 kg.m^2
%  Arm length      : 0.225 m
%  Cd (aero drag)  : 0.47
%  Max thrust      : ~44 N (4 motors)
%
%  BATTERY PARAMETERS (Part 1 - Normal)
%  -------------------------------------
%  Chemistry       : 4S LiPo
%  Capacity        : 5200 mAh
%  Nominal voltage : 14.8 V
%  Full voltage    : 16.8 V
%  Internal Res.   : 0.05 Ohm
%  Initial SoC     : 100%
%
%  BATTERY PARAMETERS (Part 2 - Degraded)
%  ----------------------------------------
%  Internal Res.   : 0.18 Ohm (3.6x degraded)
%  Initial SoC     : 40%
%  Degradation Fac : 2.8x faster discharge
%  Peukert exp.    : 1.25 (vs 1.05 normal)
%  Hard V limit    : 5.0 V  (ENFORCED)
%  Preferred V     : 6.0 V
%
%  RL PARAMETERS (Part 3)
%  -----------------------
%  Algorithm       : Tabular Q-Learning (Bellman updates)
%  State space     : Waypoint x Velocity bin x SoC bin = 75 states
%  Action space    : {Decrease, Maintain, Increase} velocity = 3 actions
%  Learning rate   : 0.15
%  Discount factor : 0.90
%  Epsilon-greedy  : 1.0 -> 0.05 (decay 0.995/episode)
%  Episodes        : 600
%
%  REWARD FUNCTION (Part 3)
%  ------------------------
%  R(time)       : -0.5 * t_leg     (penalize slow travel)
%  R(energy)     : -0.3 * E_leg     (penalize high energy)
%  R(waypoint)   : +30              (reward each intermediate WP)
%  R(arrival)    : +200             (reward reaching final WP)
%  R(lowBatt)    : -80              (SoC < 15%)
%  R(critBatt)   : -200             (V < 6V)
%  R(fastFinish) : +100             (complete with SoC > 50%)
%
%  WAYPOINTS
%  ---------
%  State 1:  [ 0,  0,  0] m  (Launch)
%  State 2:  [10,  0,  5] m
%  State 3:  [10, 10,  8] m
%  State 4:  [20, 10, 10] m
%  State 5:  [20, 20, 12] m
%  State 6:  [30, 20,  5] m  (Landing zone)
%
%  VALIDATION CHECKS (17 automated tests)
%  ----------------------------------------
%  P1: Mission completion, V>=5V, no ground penetration,
%      speed stability, SoC>=0, attitude bounded
%  P2: Hard voltage constraint, mission success, power management
%      activation, low SoC init, discharge trend, safety events
%  P3: Q-table convergence, success rate, time ratio, energy
%      positivity, reward improvement
%
%  KNOWN LIMITATIONS
%  -----------------
%  - Wind disturbances not modeled (add as Gaussian noise on forces)
%  - Motor dynamics simplified (first-order lag can be added)
%  - Euler angles have gimbal lock singularity near pitch = ±90°
%    (use quaternions for aerobatics; not needed for nominal flight)
%  - RL uses tabular Q-learning; for continuous state, switch to DQN
%    via rlDQNAgent() in the Reinforcement Learning Toolbox
%  - Simulink model (.slx) blocks require manual MATLAB Function
%    implementations after running Create_Simulink_Model.m
%
%  EXTENDING THE MODEL
%  --------------------
%  1. Add wind: apply random force in 6-DOF dynamics
%  2. Add motor saturation: clamp RPM in control allocation
%  3. Use DQN: replace Q-table with rlDQNAgent + rlMDPEnv
%  4. Sensor noise: add Gaussian noise to position/attitude states
%  5. GPS denied: integrate inertial navigation (INS) model
%
%  =========================================================================
