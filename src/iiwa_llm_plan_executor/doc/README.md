# Executing external Qwen3 graphs in Webots

This package accepts a **ready-made** JSON graph.  It does not call Qwen3, does
not evaluate the model and does not compare runs with a reference sequence.
The experiment number selects only the initial Webots scene.

## Start

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch iiwa_llm_plan_executor planner.launch.py experiment_id:=20
```

The launch starts the standard `iiwa_bringup` simulation, the writable Grant
supervisor (`mode=planner`), the scene context and the graph executor.  It
never starts `reference_runner` and never starts a second `iiwa_web` server:
the web server already comes from `iiwa_bringup` when `web.enabled` is true in
`cobot-setting.yaml`.  For a ROS-only run set `web.enabled: false` in your
settings before launch.

Useful arguments: `rviz:=true`, `config:=/path/planner.yaml`.
`simulate` must remain `true`; the physical FRI robot is not connected.

## Read the scene and submit a graph

Wait until the launch log shows the scene is ready (the snapshot is only
served after `/iiwa_experement/scene_ready`).

ROS:

```bash
ros2 service call /iiwa_scene_context/get_scene std_srvs/srv/Trigger {}
ros2 topic echo /iiwa_scene_context/scene_json --qos-durability transient_local
```

Give `scene_json` and your instruction to Qwen3 outside this project.  Submit
the answer, the `scene_revision` from the same snapshot and a client-chosen
`run_id`:

```bash
ros2 service call /iiwa_llm_plan_executor/submit webots_ros2_msgs/srv/SetString \
  "{value: '{\"run_id\":\"demo-1\",\"scene_revision\":1,\"response\":{\"status\":\"ready\",\"entry\":\"detect\",\"plan\":[...]}}'}"

ros2 topic echo /iiwa_llm_plan_executor/plan --qos-durability transient_local
ros2 topic echo /iiwa_llm_plan_executor/status
```

The accepted graph is published **unchanged** in
`/iiwa_llm_plan_executor/plan` before the first movement.  A rejected graph
never appears there; read the reason from
`/iiwa_llm_plan_executor/status` or:

```bash
ros2 service call /iiwa_llm_plan_executor/get_status iiwa_msgs/srv/GetRunStatus \
  "{run_id: demo-1}"
ros2 service call /iiwa_llm_plan_executor/cancel webots_ros2_msgs/srv/SetString \
  "{value: '{\"run_id\":\"demo-1\"}'}"
```

HTTP (existing `iiwa_web` server; default port 8007):

```bash
curl http://localhost:8007/planner/scene
curl -X POST http://localhost:8007/planner/runs \
  -H 'Content-Type: application/json' \
  -d '{"run_id":"demo-1","scene_revision":1,"response":"<Qwen JSON string>"}'
curl http://localhost:8007/planner/runs/demo-1
curl -X POST http://localhost:8007/planner/runs/demo-1/cancel
```

The same four operations are MCP tools built from the same router
(`get_scene_planner_scene_get`, `submit_run_planner_runs_post`,
`get_run_planner_runs`, `cancel_run_planner_runs`).

## Nodes and terminals

Only one graph is active per robot.  Repeating the same `run_id` is
idempotent; a different `run_id` while active is rejected (`busy`).

Status events contain the run id, node id, skill, result event, the chosen
next node, the scene revision, the mounted tool, the held object and, at the
end, the terminal.  Terminals:

* `completed` — the submitted graph and its checked postconditions finished.
* `replan_required` — the graph stopped and a fresh snapshot was emitted; the
  external client decides whether to call Qwen3 again.
* `operator_required` — a human decision is needed; the robot is stopped.
* `failed` / `canceled` — safety stop (stale state, limits, collision, cancel,
  step limit); the grasp state is preserved in the status.

The JSONL log with every event is written to
`~/.ros/iiwa_llm_plan_executor/runs/<run_id>.jsonl`.

## Safety scope

The executor checks J7 (±90° around `work`) and the downward TCP tilt (≤10°)
on **every** planned point, seeds IK from current joints, enforces these as
MoveIt path constraints, checks scene/joint freshness and verifies the final
pose.  The only logged exception is the initial empty-gripper reorientation
from home to the downward `work` posture; J7 and the final orientation are
still enforced and a reorientation with a held object is rejected.  The
executor never falls back to an unconstrained `MoveToPose` goal.  This is a
Webots verification, not a physical-robot validation and not a Qwen3 quality
evaluation.
