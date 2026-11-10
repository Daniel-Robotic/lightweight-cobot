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

`MoveItPyPlanner` uses the installed MoveItPy 2.12 API:

```python
arm.set_start_state_to_current_state()          # IK seed from current joints
arm.set_path_constraints(constraints)           # joint_7 + orientation
arm.set_goal_state(pose_stamped_msg=..., pose_link='tcp')
result = arm.plan(single_plan_parameters=params)
result.trajectory.get_robot_trajectory_msg()    # every point checked
moveit.execute(result.trajectory, controllers=[])
```

The path constraints are a `JointConstraint` on `joint_7` with
`tolerance_below/above = corridor_rad` and an `OrientationConstraint` on
`tcp` with the goal quaternion and 10 deg tolerance on each axis.  The planner
never falls back to an unconstrained `MoveToPose` goal: a rejected constrained
plan, a limit violation or an execution failure stops the backend.

