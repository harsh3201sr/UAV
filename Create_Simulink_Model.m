%% =========================================================================
%  CREATE_SIMULINK_MODEL.m
%  Programmatically creates a Simulink block diagram (.slx) for the
%  quadcopter drone system using MATLAB R2024a API
%% =========================================================================
clc; clear;

fprintf('Creating Simulink model: QuadcopterDroneSim.slx\n\n');

modelName = 'QuadcopterDroneSim';

% Close if already open
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

% Create new model
new_system(modelName);
open_system(modelName);

%% =========================================================================
%  MODEL CONFIGURATION
%% =========================================================================
set_param(modelName, ...
    'Solver',        'ode4', ...
    'FixedStep',     '0.005', ...
    'StopTime',      '120', ...
    'SolverType',    'Fixed-step', ...
    'SaveTime',      'on', ...
    'TimeSaveName',  'tout', ...
    'SaveOutput',    'on', ...
    'OutputSaveName','yout');

%% =========================================================================
%  TOP-LEVEL SUBSYSTEMS
%  Layout: [WaypointManager] -> [Controller] -> [DroneModel] -> [BatteryModel]
%                                                    ^                |
%                                               [RLAgent] <----------+
%% =========================================================================

%% -------------------------------------------------------------------------
%  SUBSYSTEM 1: Waypoint Manager
%% -------------------------------------------------------------------------
wpPos = [50, 100, 250, 200];
add_block('built-in/Subsystem', [modelName '/WaypointManager'], ...
    'Position', wpPos);

% Inside Waypoint Manager
wpSys = [modelName '/WaypointManager'];
add_block('built-in/Inport',  [wpSys '/DronePos'],  'Position', [30,  50, 60,  70]);
add_block('built-in/Outport', [wpSys '/TargetPos'], 'Position', [400, 50, 430, 70]);
add_block('built-in/Outport', [wpSys '/WP_Index'],  'Position', [400,120, 430,140], 'Port', '2');

% MATLAB Function block for waypoint logic
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [wpSys '/WP_Logic'], 'Position', [150, 40, 350, 150]);

% Add waypoint logic code
set_param([wpSys '/WP_Logic'], 'MATLABFunctionConfiguration', ...
    matlab.system.MATLABFunctionConfiguration.createFromString( ...
    ['function [target, wpIdx] = WP_Logic(dronePos)' newline ...
     '%#codegen' newline ...
     'persistent currentWP;' newline ...
     'if isempty(currentWP); currentWP = 2; end' newline ...
     'waypoints = [0,0,0; 10,0,5; 10,10,8; 20,10,10; 20,20,12; 30,20,5];' newline ...
     'err = waypoints(currentWP,:) - dronePos;' newline ...
     'if norm(err) < 0.8 && currentWP < 6' newline ...
     '  currentWP = currentWP + 1;' newline ...
     'end' newline ...
     'target = waypoints(currentWP,:);' newline ...
     'wpIdx  = currentWP;']));

add_line(wpSys, 'DronePos/1',  'WP_Logic/1', 'autorouting', 'on');
add_line(wpSys, 'WP_Logic/1',  'TargetPos/1','autorouting', 'on');
add_line(wpSys, 'WP_Logic/2',  'WP_Index/1', 'autorouting', 'on');

%% -------------------------------------------------------------------------
%  SUBSYSTEM 2: PID Controller
%% -------------------------------------------------------------------------
ctrlPos = [350, 100, 550, 200];
add_block('built-in/Subsystem', [modelName '/PIDController'], ...
    'Position', ctrlPos);

ctrlSys = [modelName '/PIDController'];
add_block('built-in/Inport',  [ctrlSys '/TargetPos'], 'Position', [30,50,60,70]);
add_block('built-in/Inport',  [ctrlSys '/DroneState'],'Position', [30,120,60,140], 'Port','2');
add_block('built-in/Inport',  [ctrlSys '/SpeedFactor'],'Position',[30,190,60,210], 'Port','3');
add_block('built-in/Outport', [ctrlSys '/CtrlOut'],   'Position', [450,80,480,100]);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [ctrlSys '/PID_Logic'], 'Position', [150,50,400,210]);

add_line(ctrlSys, 'TargetPos/1',  'PID_Logic/1', 'autorouting', 'on');
add_line(ctrlSys, 'DroneState/1', 'PID_Logic/2', 'autorouting', 'on');
add_line(ctrlSys, 'SpeedFactor/1','PID_Logic/3', 'autorouting', 'on');
add_line(ctrlSys, 'PID_Logic/1',  'CtrlOut/1',   'autorouting', 'on');

%% -------------------------------------------------------------------------
%  SUBSYSTEM 3: 6-DOF Drone Dynamics
%% -------------------------------------------------------------------------
dynPos = [650, 100, 850, 200];
add_block('built-in/Subsystem', [modelName '/DroneDynamics'], ...
    'Position', dynPos);

dynSys = [modelName '/DroneDynamics'];
add_block('built-in/Inport',  [dynSys '/CtrlInputs'], 'Position', [30,50,60,70]);
add_block('built-in/Outport', [dynSys '/DroneState'], 'Position', [450,50,480,70]);
add_block('built-in/Outport', [dynSys '/Thrust'],     'Position', [450,120,480,140], 'Port','2');

% State integrators
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [dynSys '/EOMs'], 'Position', [150,40,400,180]);

add_line(dynSys, 'CtrlInputs/1', 'EOMs/1',      'autorouting','on');
add_line(dynSys, 'EOMs/1',       'DroneState/1','autorouting','on');
add_line(dynSys, 'EOMs/2',       'Thrust/1',    'autorouting','on');

%% -------------------------------------------------------------------------
%  SUBSYSTEM 4: Battery Model
%% -------------------------------------------------------------------------
battPos = [650, 280, 850, 380];
add_block('built-in/Subsystem', [modelName '/BatteryModel'], ...
    'Position', battPos);

battSys = [modelName '/BatteryModel'];
add_block('built-in/Inport',  [battSys '/Thrust'],    'Position', [30,50,60,70]);
add_block('built-in/Inport',  [battSys '/DroneVel'],  'Position', [30,120,60,140],'Port','2');
add_block('built-in/Outport', [battSys '/SoC'],       'Position', [450,50,480,70]);
add_block('built-in/Outport', [battSys '/Vbatt'],     'Position', [450,120,480,140],'Port','2');
add_block('built-in/Outport', [battSys '/Power'],     'Position', [450,190,480,210],'Port','3');
add_block('built-in/Outport', [battSys '/SpeedFactor'],'Position',[450,260,480,280],'Port','4');

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [battSys '/Batt_Logic'], 'Position', [150,40,400,280]);

add_line(battSys, 'Thrust/1',    'Batt_Logic/1',    'autorouting','on');
add_line(battSys, 'DroneVel/1',  'Batt_Logic/2',    'autorouting','on');
add_line(battSys, 'Batt_Logic/1','SoC/1',           'autorouting','on');
add_line(battSys, 'Batt_Logic/2','Vbatt/1',         'autorouting','on');
add_line(battSys, 'Batt_Logic/3','Power/1',         'autorouting','on');
add_line(battSys, 'Batt_Logic/4','SpeedFactor/1',   'autorouting','on');

%% -------------------------------------------------------------------------
%  SUBSYSTEM 5: RL Agent (Q-Learning)
%% -------------------------------------------------------------------------
rlPos = [350, 280, 550, 380];
add_block('built-in/Subsystem', [modelName '/RLAgent'], ...
    'Position', rlPos);

rlSys = [modelName '/RLAgent'];
add_block('built-in/Inport',  [rlSys '/WP_Index'],  'Position',[30,50,60,70]);
add_block('built-in/Inport',  [rlSys '/Velocity'],  'Position',[30,120,60,140],'Port','2');
add_block('built-in/Inport',  [rlSys '/SoC'],       'Position',[30,190,60,210],'Port','3');
add_block('built-in/Outport', [rlSys '/VelAction'], 'Position',[450,80,480,100]);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [rlSys '/Q_Policy'], 'Position', [150,50,400,220]);

add_line(rlSys, 'WP_Index/1', 'Q_Policy/1',  'autorouting','on');
add_line(rlSys, 'Velocity/1', 'Q_Policy/2',  'autorouting','on');
add_line(rlSys, 'SoC/1',      'Q_Policy/3',  'autorouting','on');
add_line(rlSys, 'Q_Policy/1', 'VelAction/1', 'autorouting','on');

%% -------------------------------------------------------------------------
%  LOGGING BLOCKS (To Workspace)
%% -------------------------------------------------------------------------
logBlocks = {'SoC','Vbatt','Velocity','Position','Power'};
logPorts  = [1, 2, 1, 1, 3];     % Port numbers from BatteryModel/DroneDynamics

% Add scope
add_block('simulink/Sinks/Scope', [modelName '/MainScope'], ...
    'Position', [1000, 100, 1100, 250], ...
    'NumInputPorts', '4');

% To Workspace blocks
wsPos_start = 1000;
for i = 1:5
    wsName = [modelName '/Log_' logBlocks{i}];
    add_block('simulink/Sinks/To Workspace', wsName, ...
        'Position', [wsPos_start, 280+i*60, wsPos_start+120, 300+i*60], ...
        'VariableName', ['ws_' logBlocks{i}], ...
        'SaveFormat', 'Array');
end

%% -------------------------------------------------------------------------
%  TOP-LEVEL CONNECTIONS
%% -------------------------------------------------------------------------
% WaypointManager gets drone position from DroneDynamics
add_line(modelName, 'DroneDynamics/1', 'WaypointManager/1', 'autorouting','on');

% WaypointManager target -> Controller
add_line(modelName, 'WaypointManager/1', 'PIDController/1', 'autorouting','on');

% DroneDynamics state -> Controller
add_line(modelName, 'DroneDynamics/1', 'PIDController/2', 'autorouting','on');

% BatteryModel speed factor -> Controller
add_line(modelName, 'BatteryModel/4', 'PIDController/3', 'autorouting','on');

% Controller output -> Drone
add_line(modelName, 'PIDController/1', 'DroneDynamics/1', 'autorouting','on');

% Drone thrust & velocity -> Battery
add_line(modelName, 'DroneDynamics/2', 'BatteryModel/1', 'autorouting','on');
add_line(modelName, 'DroneDynamics/1', 'BatteryModel/2', 'autorouting','on');

% RL agent connections
add_line(modelName, 'WaypointManager/2', 'RLAgent/1', 'autorouting','on');
add_line(modelName, 'DroneDynamics/1',   'RLAgent/2', 'autorouting','on');
add_line(modelName, 'BatteryModel/1',    'RLAgent/3', 'autorouting','on');

%% -------------------------------------------------------------------------
%  ANNOTATIONS
%% -------------------------------------------------------------------------
add_block('built-in/Note', [modelName '/Note_Title'], ...
    'Position', [50, 20, 600, 50], ...
    'Text', 'QUADCOPTER DRONE SIMULATION - 6DOF | PID Control | Battery Model | Q-Learning RL');

add_block('built-in/Note', [modelName '/Note_Part1'], ...
    'Position', [50, 55, 250, 75], ...
    'Text', 'Part 1: Normal Batt | Part 2: Low Batt | Part 3: RL Opt');

%% -------------------------------------------------------------------------
%  SAVE MODEL
%% -------------------------------------------------------------------------
save_system(modelName, 'QuadcopterDroneSim.slx');
fprintf('Simulink model saved: QuadcopterDroneSim.slx\n');
fprintf('\nModel structure:\n');
fprintf('  ┌─ WaypointManager  (6 states, acceptance radius logic)\n');
fprintf('  ├─ PIDController    (Position + Attitude PID, speed scaling)\n');
fprintf('  ├─ DroneDynamics    (6-DOF EOM, mass=1.5kg, drag, gravity)\n');
fprintf('  ├─ BatteryModel     (SoC, OCV, Peukert, degradation)\n');
fprintf('  └─ RLAgent          (Q-table lookup, velocity action)\n');
fprintf('\nTo run: sim(''QuadcopterDroneSim'')\n');
fprintf('Or open GUI: open_system(''QuadcopterDroneSim'')\n');
