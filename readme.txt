Quadcopter Drone Simulation (MATLAB)

This project simulates a 1.5 kg quadcopter flying through six waypoints, with a battery model, a PID controller, and a reinforcement learning agent that tunes flight speed to save energy. It has three stages, each building on the last.

The drone and the mission
Vehicle: 1.5 kg quadcopter with a 0.225 m arm length, given inertias, aerodynamic drag, and a thrust limit of about 44 N.
Mission: fly from launch at (0,0,0) through (10,0,5), (10,10,8), (20,10,10), (20,20,12), and land at (30,20,5). Reaching a waypoint means getting within an acceptance radius (0.5 m in Part 1, 0.8 m in Part 2).
Physics: a 200 Hz loop (dt = 0.005 s) using Euler integration, a rotation matrix to convert thrust from body to world frame, and a quadratic drag force.
Control: cascaded PID
Outer loop (position): PID on X/Y/Z error. The Z output sets total thrust (Fz = m(g + u)). The X/Y outputs become desired pitch and roll angles, clamped to about 17°.
Inner loop (attitude): PID on roll, pitch and yaw error, which produces torques.
The three parts

Part 1: Normal operation. A healthy 4S LiPo (5200 mAh, 16.8 V full, 0.05 Ω internal resistance) starting at 100% charge. Power is estimated from hover power (momentum theory) plus a climb term, divided by 85% efficiency. The state of charge (SoC) is tracked by Coulomb counting with a Peukert factor, and voltage is open-circuit voltage minus the I·R drop. The goal is to show the baseline mission completes.

Part 2: Low battery. A deliberately degraded battery: 40% starting SoC, 0.18 Ω internal resistance, Peukert 1.25, and a 2.8× power multiplier. Voltage follows a nonlinear SoC-to-OCV lookup table, with a hard floor of 5 V and a preferred minimum of 6 V. A power manager reduces speed and tilt angle in two stages:

Warning (SoC < 25% or V < 10 V): speed scaled to 70%.
Critical (SoC < 15% or V < 7 V): speed scaled to 45%.

Safety events are logged, and the run ends early if the battery is depleted.

Part 3: Q-learning velocity optimization. A tabular Q-learning agent picks the cruise speed on each leg.

State: leg index (5) × velocity bin (5) × SoC bin (5) = 125 states. The README says 75, which looks like a miscount.
Actions: slow down, hold, or speed up by 1.5 m/s.
Reward: penalties for time, energy, low SoC and low voltage, with bonuses for each waypoint, for arrival, and for finishing above 50% SoC.
Training: 600 episodes, with ε-greedy exploration decaying from 1.0 to 0.05.
Evaluation: the learned policy is compared against a fixed 4 m/s baseline on mission time, energy and final SoC. This part uses a simplified leg-by-leg energy model, not the full 6-DOF simulation.
Supporting files
RunAll_DroneSimulation.m runs all three parts and builds a cross-comparison figure and summary table.
plot_drone_results.m is the shared plotting function (3D path, position, velocity, battery, dashboard).
Validate_DroneSimulation.m runs 17 pass/fail checks on the saved .mat results.
Create_Simulink_Model.m programmatically builds a Simulink diagram with five subsystems: waypoint manager, PID controller, dynamics, battery, and RL agent.
README.m documents the parameters and how to run everything.
Problems I noticed
RunAll will likely crash at the end. Each Part script begins with clear, which wipes startTime, so toc(startTime) fails. Re-create the timer after the runs, or remove clear from the Parts.
The Simulink generator is only a skeleton. The MATLAB Function blocks are empty, and matlab.system.MATLABFunctionConfiguration.createFromString isn’t a real API. Several block parameters and line connections would also likely error. The README admits the blocks need manual implementation.
The dynamics are simplified. Thrust is applied directly with no motor or propeller model, and the angular equations ignore gyroscopic coupling.
The battery checks are weak. The “voltage ≥ 5 V” test in Part 1 can never fail because Vmin is 13.2 V, and Part 2 clamps voltage to 5 V, so that “constraint” is enforced by construction rather than by the controller.
The Part 3 reward may be badly scaled. The −0.3 per Joule energy penalty is large relative to the other terms, so the agent may favor slow flight regardless of time. Check whether the “RL improves things” result is real.

I can go deeper on any part, such as the battery math, the RL formulation, or fixing the bugs above.