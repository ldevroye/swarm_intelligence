# Project context and plan

Completed work
- Read course overview and foot-bot reference: [doc/txt/argos.txt](doc/txt/argos.txt#L1) and [doc/txt/footbot.txt](doc/txt/footbot.txt#L1).
- Read exercise scenarios and solution controllers in `exercices/*/solution` (aggregation, flocking, random_walk).

Sources consulted
- ARGoS course notes: [doc/argos.pdf](doc/argos.pdf#L1) and [doc/txt/argos.txt](doc/txt/argos.txt#L1).
- Foot-bot sensor/actuator reference: [doc/footbot.pdf](doc/footbot.pdf#L1) and [doc/txt/footbot.txt](doc/txt/footbot.txt#L1).
- Lua quick reference in repo: [doc/lua.pdf](doc/lua.pdf#L1).
- Exercise solutions under `exercices/` (aggregation, flocking, random_walk).

High-level project recap
- Platform: ARGoS v3 (Lua controllers for homogeneous foot-bot swarm).
- Robot primitives used across exercises: `robot.proximity`, `robot.motor_ground`, `robot.range_and_bearing`, `robot.light`, `robot.colored_blob_omnidirectional_camera`, `robot.wheels`, `robot.random`.
- Behavior patterns present in solutions:
  - Aggregation: random walk, ground sensing (black spots), stop/leave probabilities, local communication via range-and-bearing, sleep timers to avoid oscillations.
  - Flocking / pattern formation: Lennard-Jones potential to compute inter-robot forces, optional global attractor (light/blob) to center patterns, conversion of vectors to wheel speeds.
  - Obstacle avoidance / random walk: proximity-vector summation and reactive avoidance with randomized turning.

Behavioral assumptions (tied to sources)
- Robots are identical and run the same Lua controller (course notes).
- Localized communication uses `robot.range_and_bearing` with limited payload and line-of-sight (foot-bot reference).
- Motor/proximity/ground sensor ranges and formats follow `doc/txt/footbot.txt`.

Open questions / blockers
- Final project task specification (objective, success metrics, arena layout) is not present in repo — need user to provide or confirm.
- Target number of robots and required experiments (scalability/parameters) unknown.
- Any restrictions on using C++ vs Lua controllers? Current repo already compiles a C++ experiment (`tunnelling.cpp`), but course uses Lua controllers.

Latest change
- Enabled light-sensor rays in [script.argos](script.argos) so the light readings are visible during simulation debugging.

Implementation plan (next steps)
1. Define final task and success criteria with user (choice of objective: aggregation, pattern formation, collective decision, task-specific like tunnelling).
2. Extract reusable controller primitives from exercises (modules):
	- `sensors.lua`: safe wrappers for `robot.*` access (proximity, ground, RAB, blobs, light).
	- `motion.lua`: wheel speed helpers and angle→wheel conversion.
	- `communication.lua`: RAB helpers (set/clear data, vector extraction).
3. Implement base behaviors as modular Lua components:
	- `random_walk`, `obstacle_avoidance`, `aggregation`, `lj_pattern_formation`.
4. Compose final controller from modules and tune parameters in simulation snapshots.
5. Add experiments (.argos) and scripts to run parameter sweeps and collect `output.txt` metrics.

Next immediate step (I will do now if you approve)
- Produce a concise implementation scaffold: `doc/context.md` (this file), a `controllers/` folder with the modular Lua files skeleton, and example `.argos` to run one baseline experiment (aggregation). Confirm target task first.

Developer instructions
- Only modify/create/remove Lua controllers (`.lua`) in the `controllers/` folder. Do NOT edit C/C++ source or header files (`.c`, `.cpp`, `.h`).
- The current controller to run is `controllers/tunnelling.lua`; `tunnelling.argos` is preloaded to execute it.
- Headless run (captures stdout/stderr):
	- From project root run `./start.sh 2> headless.txt`.
	- Check `headless.txt` for ARGoS prints and `[TUNNEL]` debug lines from controllers.
	- Controllers also attempt to write per-robot files named `tunnel_log_<robot-id>.log` in the simulator working directory; if absent, use `headless.txt` output for debugging.

Contacts / references
- Exercises solution files reviewed: `exercices/aggregation/solution/*`, `exercices/flocking/solution/*`, `exercices/random_walk/solution/*`.

Author: workspace analysis by assistant.

# Tunnelling controller context

## Arena and task understanding

The simulation environment defined in [tunnelling.argos](tunnelling.argos) and the loop functions in [src/tunnelling.cpp](src/tunnelling.cpp) reveal the following map situation:

- the arena is a 20 m by 20 m bounded area,
- the target region is a black rectangle centered at $(0,-2)$ with width $8$ and height $2$,
- the start/nest region is a white rectangle centered at $(0,2)$ with the same size,
- the rest of the floor is gray,
- a light source is placed above the arena at $(0,5.5,0.5)$,
- movable cylinders are placed in three obstacle bands across the arena.

The scoring logic is also clear from the loop functions:

- robots inside the black target area increase the robot count,
- obstacles inside the same area increase the obstacle count,
- the score is computed as robots minus obstacles.

## Robot capabilities available in the simulation

The foot-bot documentation confirms that the available capabilities include:

- differential-drive wheels,
- 24 proximity sensors,
- 24 light sensors,
- 4 motor-ground sensors for floor-color reading,
- a gripper actuator,
- a turret attached to the gripper,
- range-and-bearing communication.

These capabilities justify the use of a local reactive controller rather than a global planner.

## Comparison with the exercise solutions

### 1. Obstacle avoidance

The obstacle-avoidance part is directly inspired by [exercices/random_walk/solution/obstacleAvoidance_vect.lua](exercices/random_walk/solution/obstacleAvoidance_vect.lua).

Similarities:
- It sums proximity readings as vectors.
- It converts the resulting vector into an angle.
- It turns away from obstacles when the combined vector becomes significant.

Differences:
- The exercise solution uses a simple turn-based behavior with a straight-forward wheel command.
- The tunnelling controller uses a more continuous steering approach, with weighted forces and a speed reduction when an obstacle is close.

### 2. Light-based navigation

The light-following logic is inspired by the vector-based approach used in [exercices/flocking/solution/flocking.lua](exercices/flocking/solution/flocking.lua).

Similarities:
- It accumulates sensor readings into a single motion vector.
- It uses the resulting vector angle to decide the steering direction.
- It relies on a reactive, local-sensing strategy rather than a global planner.

Differences:
- The flocking exercise uses the light vector to coordinate with a swarm pattern.
- The tunnelling controller uses light information as a secondary cue and combines it with floor darkness to prefer the target side.

### 3. Floor-color target following

This is the main extension compared with the exercises. The tunnelling controller adds a floor-based dark-target heuristic:

- darker floor readings push the robot toward the target zone,
- the robot biases its steering toward darker areas,
- this creates a simple greedy behavior aligned with the task objective.

This behavior is not present in the exercise solutions and is the key adaptation for the tunnelling scenario.

### 4. Gripper and turret logic

No gripper/turret behavior is implemented yet in the current Lua script.

Current assumption tied to available sources:
- The simulator exposes gripper and turret actuators (from [tunnelling.argos](tunnelling.argos) and [doc/txt/footbot.txt](doc/txt/footbot.txt)).
- The currently available text sources do not specify a mandatory manipulation policy for Step 1.

Therefore, manipulation is postponed to a later step after baseline locomotion validation.