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

Result: the five affected packages build successfully; `colcon test` reports
`402 tests, 0 errors, 0 failures, 0 skipped` for `iiwa_msgs`,
`iiwa_experement`, `iiwa_scene_context`, `iiwa_llm_plan_executor` and
`iiwa_web`.  Four pre-existing `test_body_hatch_worlds.py` cases in other
packages still fail because their `.wbt`/`.proto` files are absent from this
checkout; they are unrelated to this change.

`git diff --check` is clean.

## Launch and scene contract

```bash
ros2 launch iiwa_llm_plan_executor planner.launch.py experiment_id:=20
```

* 19 scene objects spawned (fixture, body, cap, tray, 10 screws, 4 tools).
* `/iiwa_experement/scene_ready` true; `/iiwa_scene_context/get_scene` served
  `schema_version=2`, `contract_version=2`, `catalog_version=2`,
  `state_version=0`, live poses and action rules.  `scene_revision` is not part
  of the public envelope.
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
| invalid graph with a stray node key | `POST /planner/runs` returned 422 with `code=invalid` and the exact schema reason; `GET /planner/runs/<id>` returned 404 and no JSONL run log was created. The corrected graph with the **same** `run_id` was accepted, then `completed`. |
| different graph with an already accepted `run_id` | 409 `code=conflict`; the original run and its single `/plan` publication were unchanged. |
| identical accepted graph repeated with the same `run_id` | 200 `code=duplicate`; no second motion. |

The screw handoff graph in experiment 11 was attempted in Webots.  The tool
grab succeeded, but the subsequent screw approach was rejected by the strict
motion policy: `screw_1` failed at `tcp tilt ... 10.26 deg (limit 10.00)`, and
`screw_6` failed with `NO_IK_SOLUTION` at the far tray point.  This is the same
planning limitation recorded below; it is not a physical-robot test.  The
handoff logic itself remains covered by package tests.

## 2026-11-11 contract and pause review

### Live `scene_json`

* Experiment 20: 14 components (fixture, body, cap, tray, 10 screws), four
  tools, 14 destinations (work, hatch_seat, four holes, four sockets, four
  contour sides).  The live supervisor pose overrides the manifest pose; held
  objects are republished with their current pose and `source=webots_supervisor`.
* Experiment 8: only fixture/body, `sealant_tool`, four contour sides and that
  tool's socket; no cap, tray, screw or unrelated tool.  `dispense` is present,
  `handoff_screw` is absent.
* Experiment 11: cap, tray, 10 screws, `magnetic_tool`, four holes and its
  socket; `handoff_screw` is present.
* `docs/scene_json_experiment_20.json` is generated by the same `build_scene`
  function; a unit test fails if the example drifts.

### Dispense and tool return (experiment 8)

Graph: `grab(sealant_tool) → dispense(side_1) → transfer(sealant_tool,
sealant_tool_socket) → dock_tool(sealant_tool, sealant_tool_socket)`.
Result: `completed`.  `side_1` was an existing contour destination, the
dispense used a continuous LIN segment, and the tool was returned to its own
socket.

### Tool grab/return (experiment 20)

Graph: `grab(vacuum) → transfer(vacuum, work) → transfer(vacuum, vacuum_socket)
→ dock_tool(vacuum, vacuum_socket)`.  Result: `completed`; `mounted_tool`
ended as `null`.

### Timing before/after

The same approach/retreat graph was run in experiment 20 before and after the
planning-scene pause change.  The monotonic JSONL stages now show:

| stage | before (s) | after (s) |
| --- | --- | --- |
| inter-motion gap (`since_previous_trajectory_sec`) | 0.0015 | 0.0013 |
| `_wait_stationary` (normal) | 0.20–0.35 | 0.20–0.35 |
| PlanningScene relay | 0.10–0.21 | 0.10–0.12 |
| PlanningScene check | 0.10 | 0.10 |
| MoveIt plan | 3.6–7.5 | 3.6–7.5 |
| trajectory check | 0.002–0.004 | 0.002–0.004 |
| controller execution | 4.0–12.2 | 4.0–12.2 |
| supervisor response | 0.02–0.04 | 0.02–0.04 |

The fixed `0.5 s` wait for the next `/joint_states` sample in every full
planning-scene publish was removed; freshness is still checked immediately
before every plan.  The monitored-scene rendezvous remains because it is the
proof that MoveItPy also received the scene.  The only measured ~3 s interval
is a genuine `_wait_stationary` of 3.3–3.7 s after a dock transfer while the
Webots controller is still decelerating; it is deliberately kept because it is
the required stationary-start check, not an added sleep.  `start_delay_sec`
remains a single 3 s gate after scene load and is never applied per motion.

### Tilt limitation

The existing strict 10° tcp tilt limit rejected the far-tray screw approach in
experiment 11 (`screw_1`: 10.26°; `screw_6`: `NO_IK_SOLUTION`).  No speed or
tilt limit was raised to hide it.

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
# submit a graph through /iiwa_llm_plan_executor/submit_plan; see doc/README.md
ros2 topic echo /iiwa_llm_plan_executor/plan --qos-durability transient_local
ros2 topic echo /iiwa_llm_plan_executor/status
```

## Full Qwen cap graph (`qwen_cap_full_graph.json`)

The complete graph from the external Qwen run was added as a test fixture and
checked in scene 20.

### Empty patron

The integration test uses the real `GraphEngine`, real `SkillRegistry` and the
full 20-node graph, with fake motion/supervisor adapters that forbid supervisor
commands.  The executed node sequence is:

```
a_find_part → a_find_target → a_approach → a_pick(grasp_failed)
→ a_stop_recover → a_state(empty) → a_retry_find_part → a_retry_approach
→ a_retry_pick(grasp_failed) → stop_operator → notify → operator_required
```

The halt reasons are `required tool vacuum is not mounted`; no supervisor
command is issued, no `lower`/`contact` phase is planned, the hatch stays
`free`, and `mounted_tool`/`held_object` stay `null`.

In Webots the full graph reached the requested actual branch:

```
a_pick(grasp_failed) → a_stop_recover → a_state(empty)
→ a_retry_find_part → a_retry_approach → a_retry_pick(grasp_failed)
→ stop_operator → notify → operator_required
```

The halt reasons were `required tool vacuum is not mounted`; the supervisor
stayed at version 0, `mounted_tool`/`held_object` were `null`, the hatch stayed
`free`, and no `grab_cap`/`attach_tool`/`place_cap` command appeared in the log.

### Vacuum mounted

With `vacuum` mounted, the graph's nominal success route is:

```
a_inspect(ok) → a_state(placed) → a_inspect(ok) → ...
```

It is a cycle and the graph contains no `completed` transition at all, so a
nominal successful run cannot finish.  The full graph also has a conservative
recovery issue: `grasp_failed` leads through `inspect_state: holding` to
`a_retry_pick`, which would pick the hatch while already holding it.  The
validator therefore rejects that recovery branch with its own ownership reason
(`a_retry_pick ... while holding 'cap'`); this does not repair the model graph.
Completion must not be claimed for this graph.

## Motion planning fix (PTP/LIN, camera collision model)

### Reproduction before the change

The failing `approach(cap)` request was captured with diagnostics:

```text
phase=approach
effective_phase=transfer_lin
pipeline=pilz_industrial_motion_planner
planner_id=LIN
goal_xyz=(0.6, -0.22, 0.206667)          # base_link
goal_quat=(-0.382683, 0.92388, 0, 0)
start=(~0, ~0, ~0, ~0, ~0, ~0, ~0)
MoveIt error: NO_IK_SOLUTION
```

All three attempts failed in IK.  The old error text about the J7 corridor was
therefore misleading: the failing planner was LIN and the actual cause was
`NO_IK_SOLUTION`, not a joint-7 boundary.

### After the change

* `detect(cap) → approach(cap)` in Webots: both nodes succeeded.  Transfers
  and approaches used `pilz_industrial_motion_planner/PTP`; the measured
  minimum sampled clearance was **0.0288 m** (required 0.003 m), maximum J7
  was 0.875 rad.
* A separate J7-rotation probe moved the arm to the `work` posture with
  `joint7 = 2.0 rad` and then executed `approach(cap)`.  The accepted sampled
  trajectory covered `joint7 = -1.225 … 2.000 rad`, i.e. well beyond the old
  ±90 deg boundary, with no joint-limit, self-collision or clearance failure;
  measured clearance was again **0.0288 m**.
* Camera geometry: `camera_corner` and `camera_hand` keep at least **35 mm**
  AABB clearance to `link6`/`link7` at `home` and `work`.  The D455 collision
  box now matches its visible bounds.  Only constructive fixed contact pairs
  are disabled in `tools.yaml` and the regenerated SRDF; all camera-to-robot
  pairs remain checked.
* Tests: `pytest src/iiwa_llm_plan_executor/test` → **122 passed**; camera /
  tool collision + TCP tests in `iiwa_utils` → **7 passed**.  The existing
  unrelated
  `test_body_hatch_worlds.py` still fails because its world files are absent
  from this checkout; it is not part of the motion change.

Physical robot was not tested.
