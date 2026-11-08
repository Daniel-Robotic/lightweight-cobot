# Grant Reference Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add reproducible reference robot motion for all 20 Grant experiments, with `view` and `execute` modes in the existing ROS 2 launch command.

**Architecture:** Keep the current deterministic scene spawner. A package-owned Webots supervisor controls attachment and object poses; a ROS 2 reference runner coordinates it with MoveIt's planning scene and the existing `cobot/move_to_pose`/`cobot/move_to_named` motion server. The runner uses measured tool and task frames, short contact motions, explicit state transitions and a machine-readable event trace.

**Tech Stack:** Ubuntu 24.04, ROS 2 Jazzy, `rclpy`, MoveIt 2/`moveit_msgs`, Webots R2025a Python Supervisor, `webots_ros2_driver`, PyYAML, pytest.

**Spec:** [2026-11-08-grant-reference-control-design.md](../specs/2026-11-08-grant-reference-control-design.md)

## Global Constraints

- Work on the existing `experement` branch. Do not reset or overwrite the already modified `src/iiwa_experement/worlds/grant/iiwa-grant.wbt` or its untracked Webots sidecar files; inspect the worktree before editing.
- New runtime code, config, launch, tests and package docs live only in `src/iiwa_experement/`. Reuse OBJ/MTL from `src/iiwa_description/objects/grant/`; keep `patron` permanently installed from `src/iiwa_description/urdf/tools/patron.xacro`.
- `mode:=view` is the default and does not start robot motion. `mode:=execute` moves only after `scene_ready`, controller/MoveIt/supervisor readiness, fresh robot state, then 3.0 seconds of simulation time by default.
- The experiment is simulation-only. Reject a launch if active settings request physical FRI; preserve `simulate:=true` in the included bringup.
- Never insert or tighten screws. Release each screw at a validated handoff pose over a hole, record delivery to a virtual human, remove it from the world and leave the hole empty. The torque tool and `seat_screw`/`tighten` are absent from the reference routes.
- All free-space, approach, insertion of a *tool into patron*, hatch placement, sealant paths, and screw transport are actual robot trajectories. Only ownership changes and virtual human handoff change object state without robot motion.
- Use metres, radians and explicit `world`, `base_link`, `patron`, `tcp`, tool mount and tool work-tip transforms. Confirm the active transform at runtime; do not rely on hard-coded world-to-base coordinates from prose.
- Before every trajectory, the MoveIt scene contains every relevant obstacle and attached item. Verify planning-scene update acknowledgement before starting the next motion. Contact exceptions are specific to the active object and phase.
- Stop and report if scene geometry, collision, robot state, attachment state or action result is uncertain. No blind retries, no motion based only on startup coordinates when `detect`/current state is available.

## Read Before Editing

1. `src/iiwa_experement/config/scenes.yaml`, `iiwa_experement/{scenario.py,scene_spawner.py,webots_nodes.py,runtime_config.py}`, `launch/experiment.launch.py`, `doc/{experements.md,experiment-scenes.md,README.md}`.
2. `src/iiwa_description/urdf/tools/patron.xacro`, `src/iiwa_description/worlds/iiwa.wbt`, `src/iiwa_description/resource/protos/CollaborativeRobotTable.proto`, `src/iiwa_description/objects/grant/`, `output/reference_photo/grant/`.
3. `src/iiwa_planning/scripts/move_to_pose_server.py`, `src/iiwa_msgs/action/MoveToPose.action`, `src/iiwa_msgs/srv/MoveToNamedPose.srv`, `src/iiwa_config/config/moveit/iiwa7.srdf`, `src/iiwa_bringup/launch/supported/{moveit_nodes.py,webots_spawn.launch.py}`.
4. Existing experiment intent: `docs/superpowers/specs/2026-11-08-grant-experiments.md` and scene geometry: `docs/superpowers/specs/2026-11-08-grant-scenes.md`. The package copies take precedence after Task 1 updates screw semantics.

## File Map

| File | Responsibility |
| --- | --- |
| `config/scenes.yaml`, `iiwa_experement/scenario.py`, `doc/experements.md`, `doc/experiment-scenes.md` | Revised 20 tasks, tool sets and failure metadata; no screw seating/tightening |
| `config/geometry.yaml`, `iiwa_experement/geometry.py` | Tool mount/work tips, hatch grip and seat, contour, hole and handoff frames, measured clearances |
| `worlds/grant/iiwa-grant.wbt`, `iiwa_experement/webots_nodes.py` | Package-owned supervisor Robot and deterministic DEF/name for managed Solids |
| `iiwa_experement/grant_supervisor.py`, `iiwa_experement/object_state.py` | Webots pose following and command/state protocol, attachment state machine |
| `iiwa_experement/planning_scene.py` | World collision objects and MoveIt attach/detach updates |
| `iiwa_experement/motion.py` | Calls existing ROS motion actions/services, checks actual pose, errors and timeouts |
| `iiwa_experement/operations.py` | Reusable dock, grab, transfer, sealant, hatch and handoff operations |
| `iiwa_experement/reference_scenarios.py` | Exact task sequence and bounded recovery for IDs 1–20 |
| `iiwa_experement/reference_runner.py`, `iiwa_experement/trace.py` | Startup gate, execution, stop, JSONL events and final result |
| `iiwa_experement/runtime_config.py`, `launch/experiment.launch.py`, `setup.py`, `package.xml`, `doc/README.md` | CLI modes, package dependencies, executable wiring and instructions |
| `test/test_*.py` | Contract, geometry, launch, state, collision-scene and sequence tests |

## Review Focus

1. A launch with `mode:=view` must not create a motion client or send a trajectory; verify in Task 9.
2. `scene_ready=true` without a fresh `/joint_states` or MoveIt scene acknowledgement must not start the 3-second delay or motion; verify in Tasks 4 and 9.
3. Failed object attachment or failed MoveIt update must leave the runner stopped with no subsequent motion; verify in Tasks 3 and 4.
4. A second grab after a planned `grasp_failed` must use the current object pose and never duplicate or consume a screw; verify in Tasks 5 and 8.
5. A handoff must only succeed at the requested hole with a screw currently on the magnetic tool; no hole becomes occupied in MoveIt or Webots; verify in Tasks 3, 7 and 8.

---

### Task 1: Reconcile the experiment contract

**Files:** Modify `src/iiwa_experement/config/scenes.yaml`, `src/iiwa_experement/iiwa_experement/scenario.py`, `src/iiwa_experement/doc/experements.md`, `src/iiwa_experement/doc/experiment-scenes.md`; update affected `test/test_scenario.py`, `test/test_webots_nodes.py`, `test/test_package_layout.py`. Do not rewrite the original historical spec copies silently; add a link to this design from the package docs.

**Interfaces:** `load_scenario(experiment_id, manifest_path) -> ScenarioSpec` remains the sole source of initial poses; failure metadata remains declarative.

- [ ] **Step 1: Update failing tests.** For IDs 14–15 and 16–20 assert tool sets omit `torque_tool`; for IDs 11–20 assert exactly ten initial screws. Assert no failure entry has skill `tighten` or `seat_screw`, and that exp. 15 retains third selected screw `grab:grasp_failed`. Keep existing shifted tray/cap cases 3 and 19.
- [ ] **Step 2: Run the focused tests and observe failure.** `source /opt/ros/jazzy/setup.bash && source install/setup.bash && pytest -q src/iiwa_experement/test/test_scenario.py src/iiwa_experement/test/test_webots_nodes.py`.
- [ ] **Step 3: Revise the manifest and prose.** 11 = one handoff at `hole_1`; 12 = two diagonal; 13 = three distinct holes; 14 = four in `hole_1→hole_3→hole_2→hole_4`; 15 = four in that order with one failed third pickup and one bounded retry. 16 = one sealant segment + hatch + one handoff; 17 = full contour + hatch + two diagonal handoffs; 18 = full contour + hatch + four handoffs; 19 = same with shifted hatch/tray; 20 = same with interrupted sealant, first hatch placement `not_ok`, one failed screw pickup. Remove all tightening and moment requirements, including exp. 15's torque failure. Update `TOOL_NAMES` only if no other scene code relies on it; an unused asset may remain in `ASSETS`, but must not spawn in these routes.
- [ ] **Step 4: Run tests.** All updated scene tests pass and no package document describes a screw as installed/tightened.
- [ ] **Step 5: Commit this coherent contract change.**

### Task 2: Measure and encode the working geometry

**Files:** Create `config/geometry.yaml`, `iiwa_experement/geometry.py`, `test/test_geometry.py`; update `setup.py` if the new config glob does not already install it.

**Interfaces:** `load_geometry(path) -> GeometrySpec`; `world_target(spec: ScenarioSpec, geometry: GeometrySpec, name: str) -> Transform`; `tcp_goal(target: Transform, tcp_to_work_tip: Transform) -> Transform`. `GeometrySpec` exposes `tool_mounts`, `tool_work_tips`, `cap_grip`, `cap_seat`, `sealant_segments`, `holes`, `handoff_clearance`, `approach_clearance`, `transit_clearance`, position/angle tolerances and guarded speed scales. All coordinates use metres and radians in named frames.

- [ ] **Step 1: Write failing geometry tests.** Assert tool OBJ Y bounds become 0–0.14 m after Webots scale (sealant OBJ 0–0.014 m ×10), `tcp` in `patron.xacro` is 0.0552 m from `patron`, ten screws are distinct, holes follow body/cap shift in exp. 3, and tray/screws follow exp. 19 shift. Test a round trip `T_world_tcp · T_tcp_mount = T_world_tool_mount`.
- [ ] **Step 2: Run the test and observe failure.** `pytest -q src/iiwa_experement/test/test_geometry.py`.
- [ ] **Step 3: Measure the geometry.** Inspect OBJ vertices and reference images; use actual Webots scene tree to identify the tool's coupling end, work end, hatch grasp plane, cap seating height, sealant start/end/corners, hole axes and contact height. Store numeric transforms and conservative collision envelopes in `geometry.yaml`; retain the already documented hole XY values as cross-checks: `(0.2243,0.1052)`, `(0.2852,0.0443)`, `(0.3757,0.1348)`, `(0.3148,0.1957)` for the standard body. Do not invent unknown Z values: measure and commit them with the verification record.
- [ ] **Step 4: Implement strict loading and transform composition.** Reject missing transforms, non-finite values and nonpositive clearances. Derive shifted targets from `ScenarioSpec`, never from copied absolute coordinates.
- [ ] **Step 5: Run tests and visually check each tool aligned at `tcp`.** Save Webots screenshots/measurements or a concise QA record in `doc/README.md`.
- [ ] **Step 6: Commit geometry and measurements.**

### Task 3: Add the package-owned Webots attachment supervisor

**Files:** Modify `worlds/grant/iiwa-grant.wbt`, `iiwa_experement/webots_nodes.py`, `test/test_world.py`, `test/test_webots_nodes.py`; create `iiwa_experement/grant_supervisor.py`, `iiwa_experement/object_state.py`, `test/test_object_state.py`, `test/test_grant_supervisor.py`; update `setup.py` and `package.xml` for `std_msgs`, `webots_ros2_msgs`, `geometry_msgs`, `tf2_ros` as actually used.

**Interfaces:** A second Webots `Robot { name "GrantAttachmentSupervisor" supervisor TRUE controller "<extern>" }` in the package world; built-in `/Ros2Supervisor` remains the initial spawner. `/iiwa_experement/object_command` uses installed `webots_ros2_msgs/srv/SetString` with a JSON `value` containing `request_id`, `op`, `object`, `target`, `expected_state`; `/iiwa_experement/object_state` publishes JSON on `std_msgs/msg/String` with a monotonically increasing sequence, actual pose, holder, state and `last_request_id`. Commands: `attach_tool`, `dock_tool`, `grab_cap`, `place_cap`, `grab_screw`, `handoff_screw`, `inspect`.

- [ ] **Step 1: Write failing state tests.** Start `tool=none`, `cap=free|installed|absent`, screw set of 10; reject grab without the correct tool, second tool, second held object, wrong hole and duplicate handoff. Assert a successful handoff consumes one screw but leaves hole `empty`. Assert a failed command does not increment state sequence.
- [ ] **Step 2: Run tests and observe failure.** `pytest -q src/iiwa_experement/test/test_object_state.py src/iiwa_experement/test/test_grant_supervisor.py`.
- [ ] **Step 3: Make objects addressable.** Give every spawned managed `Solid` a stable `DEF` and preserve its current `name`. Add the invisible external supervisor Robot without replacing the table, robot or built-in `Ros2Supervisor`. Update tests that formerly asserted no `Robot {` in the template to assert exactly this one package supervisor and no preplaced iiwa or experiment object.
- [ ] **Step 4: Implement the external controller.** Follow the existing `Ros2SupervisorLauncher` environment pattern (`WEBOTS_CONTROLLER_URL` and `WEBOTS_HOME`) using a distinct robot name. Resolve named nodes after `scene_ready`, read actual `patron`/`tcp` pose from the Webots robot or a validated TF chain, and set held object pose at every Webots step using the calibrated transform. When attaching/detaching, change `locked`/physics only as needed; keep motion continuous and never teleport a held object to the task destination. Report state only after Webots confirms the change. If an object or TCP cannot be resolved, refuse the command.
- [ ] **Step 5: Run tests and a one-tool/one-cap Webots smoke test.** Check tool follows the tip and remains aligned while the arm moves; a cap follows the vacuum; docking returns the tool to the original socket. Keep 10 untouched screws stationary.
- [ ] **Step 6: Commit supervisor and state protocol.**

### Task 4: Mirror the Webots scene into MoveIt

**Files:** Create `iiwa_experement/planning_scene.py`, `test/test_planning_scene.py`; update `package.xml` with `moveit_msgs` and needed geometry dependencies.

**Interfaces:** `PlanningSceneBridge.initialize(spec, geometry)`, `attach(object_id, link_name, relative_pose, touch_links)`, `detach(object_id, world_pose)`, `remove(object_id)`, `assert_synchronized(expected_ids)`. Use `/apply_planning_scene` and `/get_planning_scene` (verify actual names in the running MoveIt configuration) or installed MoveItPy equivalent; use `moveit_msgs/msg/CollisionObject` and `AttachedCollisionObject`. The planning component in `move_to_pose_server.py` must receive and confirm the same scene before motion.

- [ ] **Step 1: Write failing scene-diff tests.** Assert fixed obstacles include table, fixture, body, rack, tray, cap, every tool in rack and all ten screws when present. Assert `attach` removes the world object and adds exactly one attached object on `tcp`/appropriate parent; `detach` reverses it; `handoff` removes only that screw. Assert allowed contact links are narrow and do not disable whole-arm/world checks.
- [ ] **Step 2: Run the focused test and observe failure.** `pytest -q src/iiwa_experement/test/test_planning_scene.py`.
- [ ] **Step 3: Implement scene updates and acknowledgement.** Derive collision primitives from `webots_nodes.py` bounds and calibrated geometry, including table top at `Z=0.793` and obstacle extents; include a safety margin in config. Apply each world/attachment diff, query it back and verify that the **separate MoveItPy planning-scene monitor in `move_to_pose_server.py`** receives it before the next motion. Inspect the active topics: the repository's `moveit_cpp.yaml` currently names `/moveit_cpp/monitored_planning_scene`, which may differ from `move_group`'s published `/monitored_planning_scene`. If they do not connect, materialize a corrected `moveit_cpp.yaml` in the package's temporary runtime directory and point the temporary `setting.yaml` at it; do not edit `iiwa_config` globally. If either Webots or MoveIt update fails, keep the runner stopped and emit a reconciliation error; never plan with mismatched scene state.
- [ ] **Step 4: Validate in RViz/Webots.** All initial objects have matching poses; attached tool and carried cap/screw move with robot in the planning scene; remaining screws stay obstacles.
- [ ] **Step 5: Commit planning-scene bridge.**

### Task 5: Build a guarded motion adapter

**Files:** Create `iiwa_experement/motion.py`, `test/test_motion.py`; update `package.xml` to depend on `iiwa_msgs`, `control_msgs`, `sensor_msgs`, `tf2_ros` as used.

**Interfaces:** `MotionAdapter.move_pose(goal: Transform, planner: Literal['ptp','lin'], speed: float, label: str) -> MotionResult`; `move_named('home', speed) -> MotionResult`; `stop(reason)`. Send to `cobot/move_to_pose`, `cobot/move_to_named`, `cobot/stop`. Goals are `tcp` poses converted to `base_link` with active TF, then to KUKA ABC (`ZYX` radians) expected by `MoveToPose.action`.

- [ ] **Step 1: Write failing fake-client tests.** No goal is sent if joint state or TF is stale, scene revision unconfirmed, `speed` outside bounds, or the robot is already stopped. A rejected/aborted/timed-out action invokes stop and prevents the next segment. A successful action checks actual TCP position/orientation within configured tolerances.
- [ ] **Step 2: Run tests and observe failure.** `pytest -q src/iiwa_experement/test/test_motion.py`.
- [ ] **Step 3: Implement the adapter.** Use `ptp`/OMPL only for free-space transit with full collision scene, `lin` for short axial approach/retreat and contour segments; bound planning and execution timeouts, check fresh `/joint_states`, actual `tcp` TF and controller readiness. Do not use `iiwa_planning/direct_trajectory.py` or TCP gizmo for the baseline; disable the gizmo in the per-launch settings during `execute` so there is one command source.
- [ ] **Step 4: Execute a motion-only Webots smoke test.** From home, go to a measured clearance pose over the empty work zone and return home with no contact; confirm actual path and joint limits.
- [ ] **Step 5: Commit motion adapter.**

### Task 6: Implement reusable tool, hatch and sealant operations

**Files:** Create `iiwa_experement/operations.py`, `test/test_operations.py`.

**Interfaces:** `take_tool(tool_name)`, `dock_tool(tool_name)`, `pick_cap()`, `place_cap(target_pose)`, `dispense(segment_name, start_fraction=0.0)`, `inspect_cap()`, `inspect_sealant()`. `take_tool` inserts a tool into `patron`; `dock_tool` returns it to its socket. Each returns a typed outcome and emits trace events; the supervisor/object-state bridge and MoveIt scene bridge are the only state writers.

- [ ] **Step 1: Write failing operation-sequence tests.** Assert `home/clearance→tool preapproach→axial insertion to mount→attach→axial extraction` for `take_tool`; reverse for `dock_tool`. Assert cap route `pregrasp→lower→grab→lift→clearance transfer→align→lower to seat→place/release→retreat`; verify cap motion tracks TCP and no direct cap teleport. Assert `dispense` moves the nozzle along the actual four 3D sides, preserves working height through adjacent corners and changes a side to complete only after the full motion succeeds.
- [ ] **Step 2: Run tests and observe failure.** `pytest -q src/iiwa_experement/test/test_operations.py`.
- [ ] **Step 3: Implement operations with calibrated transforms.** Compute each TCP target from the named work-tip target. Use local target axes for approach/retreat; compute transfer clearance from the carried envelope and obstacles, not one global magic Z. Before grab or release, require measured proximity and expected object state. For exp. 5 support one deliberately offset placement followed by pickup and correction. For exp. 9/20 record last completed sealant fraction and resume from there.
- [ ] **Step 4: Run unit tests and Webots smoke tests for experiments 1, 6, 9 and 10.** Observe real robot motion with tool, cap and nozzle; no object jumps directly to a target.
- [ ] **Step 5: Commit tool/hatch/sealant operations.**

### Task 7: Implement magnetic pickup and virtual handoff

**Files:** Extend `iiwa_experement/operations.py`, `iiwa_experement/object_state.py`, `test/test_operations.py`, `test/test_object_state.py`.

**Interfaces:** `pick_screw(screw_id)`, `handoff_screw(screw_id, hole_id)`. The final event is `delivered(screw_id,hole_id)`, never `seated` or `tightened`.

- [ ] **Step 1: Write failing tests.** A screw can be selected from any of ten available IDs; held screw follows magnetic work tip; approach the selected hole on its axis; `handoff_screw` fails unless the actual robot TCP is within the measured handoff pose tolerance and the screw is held. Successful handoff removes that single screw from Webots and MoveIt, leaves the hole empty, and leaves nine/then fewer untouched screws in the tray.
- [ ] **Step 2: Run tests and observe failure.** `pytest -q src/iiwa_experement/test/test_operations.py src/iiwa_experement/test/test_object_state.py`.
- [ ] **Step 3: Implement actual path.** `pregrasp over selected screw→axial lower→grab→axial lift→clearance transfer→pre-handoff above hole→axis alignment→short lower to handoff pose→release to virtual human→axial retreat`. Do not command screw penetration into cap/body. Between holes, move through the verified clearance corridor, never laterally across the cap at contact height.
- [ ] **Step 4: Run Webots smoke tests for 11, 12, 14.** Verify each tool/screw visibly travels with robot; all ten are initially present and only delivered screws disappear.
- [ ] **Step 5: Commit handoff operation.**

### Task 8: Define the 20 reference sequences and bounded recovery

**Files:** Create `iiwa_experement/reference_scenarios.py`, `test/test_reference_scenarios.py`; extend `iiwa_experement/operations.py` for `inspect_state` recovery as needed.

**Interfaces:** `build_reference_sequence(spec: ScenarioSpec) -> tuple[Operation, ...]`; `execute_sequence(sequence, context) -> RunResult`. Sequence names/targets correspond to the package `doc/experements.md`; every successful sequence ends by returning the current tool and moving to `home`.

- [ ] **Step 1: Write failing table-driven tests for IDs 1–20.** Assert exact operation order by category from Task 1, `dock_tool` before every switch, unique selected screws and required hole count. For 3/19 use actual shifted poses. For 4/15/20 verify one `grasp_failed` triggers `stop→inspect_state→redetect→one retry`; for 5 verify one `not_ok` triggers cap correction; for 9/20 verify resume at the missing sealant fraction. A second failure or unknown state ends `stopped` with no later motion. No trace contains `seat_screw` or `tighten`.
- [ ] **Step 2: Run tests and observe failure.** `pytest -q src/iiwa_experement/test/test_reference_scenarios.py`.
- [ ] **Step 3: Implement sequence selection and fault injection.** Consume `ScenarioSpec.failures` deterministically by matching skill, target and attempt; do not mutate the initial scene in `view`. Recovery reads actual simulator state before deciding whether to retry, skip an already completed transfer or stop.
- [ ] **Step 4: Run focused tests and manually inspect expected sequence listing for all 20 IDs.** Distinguish three screw handoffs in exp. 13 from four in 14/15; exp. 19 must use shifted cap/tray.
- [ ] **Step 5: Commit scenario runner.**

### Task 9: Wire `view` and `execute` into the existing command

**Files:** Modify `iiwa_experement/runtime_config.py`, `launch/experiment.launch.py`, `setup.py`, `package.xml`, `test/test_launch.py`, `test/test_runtime_config.py`; create `iiwa_experement/reference_runner.py`, `iiwa_experement/trace.py`, `test/test_reference_runner.py`.

**Interfaces:** CLI `ros2 launch iiwa_experement experiment.launch.py experiment_id:=N mode:=view|execute start_delay_sec:=3.0 trace_dir:=~/.ros/iiwa_experement/runs`; default `mode=view`, default trace directory as shown. `reference_runner` executable only in `execute`; `grant_supervisor` executable in both modes if needed for state inspection, but it must never change objects in `view`. `RunResult` and JSONL trace expose experiment ID, simulation timestamp, operation, object, target, start/end pose, outcome and state transition.

- [ ] **Step 1: Write failing launch/startup tests.** Check strict mode validation and finite nonnegative delay before Webots starts; `view` produces only bringup + scene spawner (+ inert supervisor) and zero motion clients; `execute` adds supervisor and runner. Check `scene_ready=false`, absent MoveIt/controller, stale joint states or paused `/clock` cannot start motion. Once all ready, the first motion starts no earlier than `scene_ready` time + `start_delay_sec` simulation seconds. On readiness timeout, exit nonzero with no robot goal.
- [ ] **Step 2: Run tests and observe failure.** `pytest -q src/iiwa_experement/test/test_launch.py src/iiwa_experement/test/test_reference_runner.py`.
- [ ] **Step 3: Implement startup and trace.** Wait for `scene_ready`, supervisor, MoveIt, relevant action/service servers and fresh TF/joint states, then start sim-time delay. Publish a concise progress topic and write JSONL to `trace_dir/experiment_<id>_<wall-clock-UTC>.jsonl`, expanding `~` in the default `~/.ros/iiwa_experement/runs`; flush after every operation so stopped runs retain evidence. On success or failure publish final `RunResult`; do not automatically quit Webots so the user can inspect the final scene.
- [ ] **Step 4: Run launch tests and manual mode checks.** `view` shows the unchanged scene indefinitely. `execute` waits three sim seconds after readiness, then visibly moves. Check `start_delay_sec:=0` and a longer delay (e.g. 10) as boundary cases.
- [ ] **Step 5: Commit launch and tracing.**

### Task 10: Validate safety, the 20 runs and handoff documentation

**Files:** Modify `src/iiwa_experement/doc/README.md`; add `src/iiwa_experement/doc/reference-control-qa.md` with measured transforms, collision checks and per-experiment results. Update tests only to cover concrete issues found during simulation.

**Interfaces:** The public command and trace format from Task 9 are the handoff contract for the user's LLM plan comparison.

- [ ] **Step 1: Build and run package tests.** `source /opt/ros/jazzy/setup.bash && colcon build --base-paths src --packages-up-to iiwa_experement --symlink-install`; then `source install/setup.bash && colcon test --base-paths src --packages-select iiwa_experement && colcon test-result --verbose`. Fix failures before claiming completion.
- [ ] **Step 2: Run `view` for representative IDs 1, 11, 16, 19 and 20.** Check object counts, 10 tray screws, one permanent `patron`, no arm movement and no object change after at least 10 sim seconds.
- [ ] **Step 3: Run `execute` for IDs 1–20 individually.** Capture for each: actual motion visible, TCP/tool alignment, operation trace, no forbidden collisions/penetrations, final cap pose, correct sealant segments, expected distinct handoffs, returned tool and home. If a measured pose is unreachable or unsafe, correct geometry/scene representation and repeat that ID plus affected neighbours; never bypass the collision check.
- [ ] **Step 4: Exercise fault routes 4, 5, 9, 15, 20.** Verify exactly one recovery attempt at the intended component and stop on repeated or unknown failure. Verify the screw is never inserted, the hole remains empty after handoff, and no torque tool motion occurs.
- [ ] **Step 5: Document exact launch examples, parameters, trace schema, numerical calibration, test results, unresolved limits and simulation-only status.** Include `ros2 launch iiwa_experement experiment.launch.py experiment_id:=1 mode:=view` and `... mode:=execute start_delay_sec:=3.0`.
- [ ] **Step 6: Commit documentation and final verified changes.** Review `git diff` and preserve unrelated changes already in the worktree.

## Completion Criteria

The work is complete only when `view` remains stationary, `execute` moves the robot through all intended operations in all 20 scenarios, the tool is visibly attached at the patron tip, the hatch moves with the robot and ends on the body, sealant follows named paths, each screw physically travels to a hole and is released to the virtual human without seating or tightening, and build/tests plus recorded Webots smoke checks pass. A failed or unsafe scenario is reported as incomplete rather than silently marked successful.
