# Executing external Qwen3 graphs in Webots

## Краткая инструкция (русский)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch iiwa_llm_plan_executor planner.launch.py experiment_id:=20
```

Дождитесь готовности сцены, прочитайте `/planner/scene` или
`/iiwa_scene_context/get_scene`, передайте `scene_json` и инструкцию Qwen3
снаружи, затем отправьте готовый граф в
`/iiwa_llm_plan_executor/submit_plan` (`iiwa_msgs/SubmitPlan`) или через
`POST /planner/runs` с телом `{"run_id": "...", "response": {...}}`.
`scene_revision` не передаётся и не возвращается. Отклонённый запрос не
регистрируется, поэтому тот же `run_id` можно использовать после исправления
графа. Подробности: `docs/interfaces.md`.

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
the answer and a client-chosen `run_id`; the executor obtains and validates its
own fresh scene snapshot, so no scene revision travels through the API:

```bash
ros2 service call /iiwa_llm_plan_executor/submit_plan iiwa_msgs/srv/SubmitPlan \
  "{run_id: 'demo-1', response: '{\"status\":\"ready\",\"entry\":\"detect\",\"plan\":[...]}'}"

ros2 topic echo /iiwa_llm_plan_executor/plan --qos-durability transient_local
ros2 topic echo /iiwa_llm_plan_executor/status
```

The legacy `/iiwa_llm_plan_executor/submit` (`webots_ros2_msgs/SetString`) still
routes into the same admission path.  Rejections return `code` and `reason`
from `SubmitPlan`; a rejected `run_id` is not registered and can be reused with
a corrected graph.

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
  -d '{"run_id":"demo-1","response":"<Qwen JSON string>"}'
curl http://localhost:8007/planner/runs/demo-1
curl -X POST http://localhost:8007/planner/runs/demo-1/cancel
```

The same four operations are MCP tools built from the same router
(`get_scene_planner_scene_get`, `submit_run_planner_runs_post`,
`get_run_planner_runs`, `cancel_run_planner_runs`).

## Nodes and terminals

Only one graph is active per robot.  Repeating an identical accepted graph is
idempotent (`duplicate`); another graph with the same accepted `run_id` is a
`conflict`; a different `run_id` while a run is active is rejected (`busy`).
Accepted run ids stay bound after every terminal state.

Status events contain the run id, node id, skill, result event, the chosen
next node, the mounted tool, the held object and, at the end, the terminal.
The internal scene revision is never exposed.  Terminals:

* `completed` — the submitted graph and its checked postconditions finished.
* `replan_required` — the graph stopped and a fresh snapshot was emitted; the
  external client decides whether to call Qwen3 again.
* `operator_required` — a human decision is needed; the robot is stopped.
* `failed` / `canceled` — safety stop (stale state, limits, collision, cancel,
  step limit); the grasp state is preserved in the status.

The JSONL log with every event is written to
`~/.ros/iiwa_llm_plan_executor/runs/<run_id>.jsonl`.

## Safety scope

The executor uses Pilz PTP for free-space working-point moves and Pilz LIN for
axial contact/retreat and the dispense contour.  Every plan is densely
resampled and checked for real joint limits, self-collision (including the
camera assembly), scene collision, camera clearance, downward TCP tilt (≤10°)
and joint jumps before execution.  J7 may use its full safe joint range; the
`work` value is only a preference.  A lateral PTP from a contact pose is
preceded by an axial LIN retreat.  The only logged orientation exception is
the initial empty-gripper reorientation from home to the downward `work`
posture.  The executor never falls back to an unconstrained `MoveToPose` goal.
This is a Webots verification, not a physical-robot validation and not a
Qwen3 quality evaluation.
