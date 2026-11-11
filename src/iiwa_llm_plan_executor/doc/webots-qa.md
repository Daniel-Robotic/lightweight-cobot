# Webots QA: external graph execution

Scope: Webots only (`simulate:=true`).  No physical FRI robot was connected and
no Qwen3 quality/score system was built or called.

## Build and package tests

Commands:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install \
  --packages-up-to iiwa_experement iiwa_scene_context iiwa_llm_plan_executor iiwa_web
source install/setup.bash
colcon test --base-paths src --packages-select \
  iiwa_experement iiwa_scene_context iiwa_llm_plan_executor iiwa_web
colcon test-result --verbose
```

Result: build of 10 packages succeeded; `359 tests, 0 errors, 0 failures,
0 skipped`.

`git diff --check` is clean.

## Launch and scene contract

```bash
ros2 launch iiwa_llm_plan_executor planner.launch.py experiment_id:=20
```

* 19 scene objects spawned (fixture, body, cap, tray, 10 screws, 4 tools).
* `/iiwa_experement/scene_ready` true; `/iiwa_scene_context/get_scene` served
  `contract_version=1`, `catalog_version=1`, `scene_revision=1`,
  `state_version=0` with all present ids and no coordinates.
* The camera contract was corrected to the topics actually published by the
  existing Webots camera plugins: `/d455_top/image_raw/image_color` and
  `/camera/color/image_color`.
* `/iiwa_web` was started once by `iiwa_bringup` (`web.enabled: true`); the
  planner launch did not start a second web server.

## Graphs exercised in scene 20

All graphs were submitted through `/iiwa_llm_plan_executor/submit` with a
client-supplied `run_id`; the accepted graph appeared in
`/iiwa_llm_plan_executor/plan` before the first movement.

| Graph | Result |
| --- | --- |
| `detect(cap)` | `completed`; `found`, cap pose read from supervisor, fresh top+hand frames. |
| `grab(vacuum) → transfer(vacuum,work) → completed` | `completed`; status ended with `mounted_tool: "vacuum"`, `held_object: null`. The cup stayed in `patron`. |
| `transfer(vacuum,vacuum_socket) → dock_tool(vacuum,vacuum_socket) → completed` | `completed`; status ended with `mounted_tool: null`. This was a separate graph after the previous one. |
| `grab(cap) → transfer(cap,hatch_seat) → align → release(cap,hatch_seat)` (cup already mounted) | Cap grasp, transfer, align and release succeeded; the supervisor reported the hatch installed. The final `verify` first failed because of a web-wrapper attribute shadowing bug (`latest`), which was fixed afterwards; a clean repeat of the whole cap graph was not obtained in this session. |
| cap graph without explicit `grab(vacuum)`, with a `grasp_failed` edge | Admitted; execution returned `grasp_failed` with `reason: "required tool vacuum is not mounted"`, `next: "operator_required"`, `mounted_tool: null`, `held_object: null`. The supervisor state stayed at version 0 with the cap `free` and `vacuum` `docked`; the log contains no planner call for that run. `/plan` held the original graph, `/status`, `GET /planner/runs/...` and the JSONL log held the actual event/reason/ownership/next edge. |
| stale `scene_revision=999` | Rejected: `stale_scene: submitted revision 999 does not match current revision 1`. |

The screw handoff graph (`grab(magnetic_tool) → grab(screw_1) →
transfer(screw_1,hole_1) → handoff_screw`) is covered by package tests but was
not completed in this Webots session because the preceding cap/cartesian
planner runs were not repeatable within the remaining time.

## J7 and tilt evidence

Every executed trajectory passed `MotionPolicy` before the controller. The
JSONL log records e.g.:

```json
{"event":"motion_planned","phase":"transfer","points":64,
 "min_j7":-3.2e-07,"max_j7":0.874,"max_tilt_deg":180.0,
 "reorientation":true,"max_joint_jump_rad":0.0349,
 "max_cartesian_step_m":0.0387}
```

That is the explicit empty-gripper reorientation from home (tip up) to the
downward `work` posture.  Working motions keep J7 inside the ±90 deg corridor
and are rejected point-by-point when J7, tilt or jumps leave the configured
bounds.  Out-of-corridor and stale-state graphs stopped without execution.

## Remaining limitations found in Webots

1. **OMPL goal sampling is not deterministic.** With both the J7 corridor and a
   strict orientation goal region, OMPL reports “more than 80% of sampled goal
   states fail to satisfy the constraints” and sometimes returns no plan. The
   executor retries constrained plans, but identical graphs can succeed in one
   run and return `replan_required` in another.
2. **Pilz does not honor path constraints in this installation.** PTP can pick
   the forbidden J7 branch; LIN with non-empty path constraints can return no
   trajectory. The adapter therefore uses OMPL+J7 for tool transits, LIN for
   part transits/approaches/contact, and enforces all tilt/J7/jump limits after
   planning. This is a planning limitation, not a fallback to an unconstrained
   `MoveToPose` action.
3. **Tilt limit.** The original Webots runs returned some OMPL/LIN paths at
   10.03–10.16 deg tilt against the 10 deg limit; the previous 0.25 deg
   allowance accepted them. The review on 2026-11-11 replaced that allowance
   with floating-point roundoff only, so those paths are now rejected before
   execution. This stricter policy passed package tests but has not yet been
   rechecked in Webots. A planner/adapter that honors the orientation
   constraint end-to-end is still needed for reliable execution.
4. **Controller start overshoot.** Webots can settle a joint ~3e-5 rad outside
   MoveIt's soft limit; the start state is now clamped to the same
   `soft_min_position`/`soft_max_position` band before planning.
5. **Cancel and collision in Webots.** Cancel is covered by engine/node tests
   (active run stops, grasp preserved). The Webots cancel probe attempted here
   ended in a planner `replan_required` before the cancel instant, so it is not
   counted as a successful Webots cancel demonstration. No synthetic collision
   was injected; collision branches are covered by validator/back-end tests and
   any planner/execution failure stops further motion.
6. This is not a physical-robot validation and not an evaluation of Qwen3.

## Reproduce

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch iiwa_llm_plan_executor planner.launch.py experiment_id:=20
ros2 service call /iiwa_scene_context/get_scene std_srvs/srv/Trigger {}
# submit a graph through /iiwa_llm_plan_executor/submit; see doc/README.md
ros2 topic echo /iiwa_llm_plan_executor/plan --qos-durability transient_local
ros2 topic echo /iiwa_llm_plan_executor/status
```
