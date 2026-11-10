# `iiwa_llm_plan_executor` interfaces

Graph contract version: **1** (it matches `iiwa_scene_context` contract 1).
The executor never calls Qwen3 and never rewrites the model graph.

## Graph response

The accepted response is exactly:

```json
{
  "status": "ready",
  "entry": "p1_find",
  "plan": [
    {
      "id": "p1_find",
      "skill": "detect",
      "object": "cap",
      "target": "",
      "params": {},
      "transitions": [
        {"event": "found", "next": "p1_grab"},
        {"event": "not_found", "next": "replan_required"},
        {"event": "failed", "next": "replan_required"}
      ]
    }
  ]
}
```

* `status` is `ready` or `blocked`. `blocked` is never executed.
* `plan` is a list of at most 256 nodes. Unknown keys, duplicate ids,
  reserved terminal ids, missing `entry`, missing required events, unknown
  events, unknown `next`, unreachable nodes and nodes that cannot reach a
  terminal are rejected.
* Transitions may also be sent once at top level as
  `{"from": "...", "event": "...", "to": "..."}`; they are merged into the
  node by `parse_response`. Mixing both forms for the same event is rejected.
* Reserved terminals: `completed`, `replan_required`, `operator_required`.
  `completed` is the verified end of the submitted graph; it is not a score of
  the model against a Russian instruction.
* `params` is optional and currently only `dispense` accepts
  `start_fraction`/`end_fraction` in `0..1`.

## Skills and aliases

Native: `detect`, `approach`, `lower`, `grab`, `lift`, `transfer`, `align`,
`release`, `retreat`, `capture`, `verify`, `stop`, `wait_scene_clear`,
`inspect_state`, `notify_operator`, `dispense`, `dock_tool`,
`handoff_screw`.

Compatibility aliases: `pick` (= `grab`), `move_to_storage`
(= `transfer` + axial `lower`, keeping the grasp), `inspect` (= `verify`).
`tighten` is not in this profile and is rejected before any motion.

The contract, outcomes and available ids come from
`iiwa_scene_context/catalog.py` and the live `scene_json`; a model cannot add
a skill or a contract.

## Admission rules

`parse_response(raw) -> PlanGraph` performs the strict JSON shape check.
`validate_graph(graph, scene_envelope, catalog, expected_scene_revision=...)`
returns `ValidationResult(ok, code, reason, details)` with codes:

| code | meaning |
| --- | --- |
| `ok` | admitted |
| `blocked` | the model returned `status=blocked`; no motion |
| `replan_required` | a fresh scene/plan is required before motion (missing tool, missing explicit transfer, unconfirmed ownership) |
| `rejected` | structural or safety violation |
| `stale_scene` | the graph was built for another `scene_revision` |

Important pre-movement rules:

* `grab(cap)` requires `vacuum` in `patron`. If the scene starts with the cup
  docked, the graph must explicitly `grab(vacuum)`; otherwise admission is
  `replan_required` and nothing moves.
* `release(vacuum)` without a target, or anywhere except its named socket and
  after an explicit `transfer(vacuum, vacuum_socket)`, is `replan_required`.
  The cup stays mounted. `release(vacuum, vacuum_socket)` and
  `dock_tool(vacuum, vacuum_socket)` are accepted only at the reached socket.
* A graph ending in `work` with `vacuum` mounted is accepted; the tool stays in
  `patron`.
* `grasp_failed` must go to `inspect_state` or `stop`; it can never become
  `completed`. `collision` must reach `stop`/`notify_operator` or
  `replan_required`/`operator_required`.
* Screws are handed to the virtual operator at a hole; the hole stays empty
  and no insertion/tightening occurs.
* No task is inferred from `experiment_id`; only the initial scene is selected.

## ROS execution interfaces

| Direction | Name | Type | Payload |
| --- | --- | --- | --- |
| service | `/iiwa_llm_plan_executor/submit` | `webots_ros2_msgs/SetString` | `{"run_id", "scene_revision", "response"}`. `success` means admitted; the rejection reason is in `/status` and `get_status`. |
| topic | `/iiwa_llm_plan_executor/plan` | `std_msgs/String` | `{"run_id","scene_revision","response"}` exactly for the accepted graph; transient-local, depth 1. Published once per `run_id` before the first node. Rejected graphs never appear. |
| topic | `/iiwa_llm_plan_executor/status` | `std_msgs/String` | Each node event and terminal. Fields: `run_id`, `state`, `reason`, `node_id`, `skill`, `event`, `next`, `scene_revision`, `terminal`, `mounted_tool`, `held_object`, `timestamp`, plus event details. |
| service | `/iiwa_llm_plan_executor/get_status` | `iiwa_msgs/GetRunStatus` | request `string run_id`; response `bool found`, `string status_json`. |
| service | `/iiwa_llm_plan_executor/cancel` | `webots_ros2_msgs/SetString` | `{"run_id": "..."}` or a plain run id; stops execution and preserves the grasp. |

Admission conditions: fresh `scene_json`, matching `scene_revision`,
`scene_ready`, a ready MoveIt/controller, fresh `joint_states`, and at least
3.0 s of simulation time since the first ready scene. Only one run may be
active. Repeating `submit` with the same `run_id` is idempotent and never
starts a second motion; a different `run_id` while active is rejected as
`busy`.

Terminals are `completed`, `replan_required`, `operator_required`, plus
`failed`/`canceled` for safety stops. `replan_required` emits the newest
`scene_json`; the external client decides whether to call Qwen3 again.
Execution runs in a worker thread, so `get_status` and `cancel` stay
responsive. A JSONL log with every event is written under `trace_dir`
(default `~/.ros/iiwa_llm_plan_executor/runs`).


## Skill phases

A skill may contain the mandatory geometric phases of one action. `approach`
moves above the target and saves that height; `lower` moves along the tool
axis to contact; `grab` performs only the missing phases, confirms the grasp
through the supervisor, then lifts back to the saved approach pose. Explicit
`approach`/`lower`/`lift` nodes do not repeat motion already performed.

`release` always lifts axially before any lateral motion or wrist rotation.
`transfer` stops at the safe height over the destination. `release(cap,
hatch_seat)` lowers, places, detaches and withdraws. `handoff_screw` lowers at
the hole, hands the screw to the virtual operator and withdraws; it never
inserts or tightens and the hole remains empty.

`detect` first withdraws axially from a contact, moves to `work`, waits for a
fresh top frame and reads the exact pose from the supervisor. When the object
is configured for close inspection, a second fresh frame from the hand camera
is required; RGB/D is used for liveness only.


## Constrained motion adapter

`MotionPolicy` (`config/motion_limits.yaml`) checks every point before the
controller sees a plan:

* `joint_7` must stay inside `work_reference_rad +/- corridor_rad`
  (0 +/- 90 deg; boundaries inclusive).
* The tcp local +Z (working direction) must point down within
  `max_tilt_deg` (10 deg).
* Consecutive points may not jump more than `max_joint_jump_rad` (0.6 rad) or
  `max_cartesian_jump_m` (0.15 m); the first point must match the measured
  joints used as IK seed.
* `joint_states` and the planning scene freshness are checked before planning.

One explicit exception is recorded: if the measured start posture is not yet the
downward working posture (the calibrated home pose points the tip up) and the
gripper is empty, the first reorientation to `work` is allowed to pass through
larger tilt angles.  Joint 7, joint jumps, Cartesian jumps and the final
downward orientation are still checked, and the event records
`reorientation: true`.  A reorientation with a held object is always rejected.

`MoveItPyPlanner` uses the installed MoveItPy 2.12 API:

```python
arm.set_start_state(robot_state=clamped_robot_state)  # measured joints as IK seed
arm.set_path_constraints(constraints)                 # hard joint_7 corridor
arm.set_goal_state(pose_stamped_msg=..., pose_link='tcp')
result = arm.plan(single_plan_parameters=params)
result.trajectory.get_robot_trajectory_msg()          # every point checked
moveit.execute(result.trajectory, controllers=[])
```

The path constraint is a `JointConstraint` on `joint7` with
`tolerance_below/above = corridor_rad`.  The planner applies it on OMPL
transfers.  In this MoveIt/Pilz installation the orientation path constraint is
not honored reliably, so the 10 deg working-tilt limit is enforced **after**
planning by `MotionPolicy` on every returned point; the planner also gets a
wider internal orientation tolerance for its goal sampling.  Tool transits use
OMPL with the J7 corridor, part transits and approaches use Cartesian LIN, and
short contact moves use LIN.  A plan that cannot be produced or that violates
the point-wise checks is stopped; there is no unconstrained `MoveToPose`
fallback.

The measured start state is clamped to the same soft joint limits used by
MoveIt (`soft_min_position`/`soft_max_position`) before it becomes the IK
seed.  Without that clamp a Webots-controller overshoot of ~0.00003 rad makes
`CheckStartStateBounds` abort every following plan.

