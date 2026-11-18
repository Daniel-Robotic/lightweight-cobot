# Executing external graph-v3 plans in Webots

## Краткая инструкция (русский)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch iiwa_llm_plan_executor planner.launch.py experiment_id:=20
```

Дождитесь готовности сцены и запросите `GET /planner/context`. Пакет уже
содержит system prompt, публичную сцену v1, каталог навыков, JSON Schema
graph-v3 и `state_version`. Передайте его вместе с задачей оператора во
внешний клиент модели, затем отправьте ответ в `POST /planner/runs`:

```json
{"run_id": "...", "context_state_version": "...", "response": {...}}
```

`response` — нетрансформированный graph-v3. Старый формат без
`schema_version: 3` отклоняется. Отклонённый запрос не регистрируется,
поэтому тот же `run_id` можно использовать после исправления графа.
Подробности: `docs/interfaces.md`.

This package accepts a **ready-made graph-v3** response.  It does not call the
model, does not evaluate its quality and does not compare runs with a reference
sequence.  The experiment number selects only the initial Webots scene.

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

## Read the context and submit graph-v3

Wait until the launch log shows the scene is ready (the snapshot is only
served after `/iiwa_experement/scene_ready`).  Build one model-input package
from the live snapshot through the HTTP API:

```bash
curl http://localhost:8007/planner/context
```

The package contains `system_prompt`, `scene`, `catalog`, `response_schema`,
`schema_version=3`, `contract_version`, `catalog_version` and
`state_version`.  Combine the prompt, scene, catalog and your task in the
external model client, call your own model, and submit its graph-v3 answer:

```bash
curl -X POST http://localhost:8007/planner/runs \
  -H 'Content-Type: application/json' \
  -d '{"run_id":"demo-1","context_state_version":"7","response":{...}}'
```

The ROS service carries the same fields:

```bash
ros2 service call /iiwa_llm_plan_executor/submit_plan iiwa_msgs/srv/SubmitPlan \
  "{run_id: 'demo-1', context_state_version: '7', response: '<graph-v3 JSON>'}"
```

`GET /planner/scene` and `/iiwa_scene_context/get_scene` remain diagnostic
scene-v2 endpoints; they do not contain the model prompt or graph schema.
Rejections return `code`, `reason` and `details_json`; a rejected `run_id` is
not registered and can be reused with a corrected graph.  The old executor
graph format is rejected as `unsupported_contract`.

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
curl http://localhost:8007/planner/context
curl -X POST http://localhost:8007/planner/runs \
  -H 'Content-Type: application/json' \
  -d '{"run_id":"demo-1","context_state_version":"7","response":"<graph-v3 JSON>"}'
curl http://localhost:8007/planner/runs/demo-1
curl -X POST http://localhost:8007/planner/runs/demo-1/cancel
```

The same operations are MCP tools built from the router
(`get_context_planner_context_get`, `get_scene_planner_scene_get`,
`submit_run_planner_runs_post`, `get_run_planner_runs`,
`cancel_run_planner_runs`).

## Nodes and terminals

Only one graph is active per robot.  Repeating an identical accepted graph is
idempotent (`duplicate`); another graph with the same accepted `run_id` is a
`conflict`; a different `run_id` while a run is active is rejected (`busy`).
Accepted run ids stay bound after every terminal state.

Status events contain the run id, node id, skill, result event, the chosen
next node, the mounted tool, the held object and, at the end, the terminal.
The internal scene revision is never exposed.  Terminals:

* `completed` — the graph reached the terminal only after every `goals` predicate was confirmed by a live observation.
* `replan_required` — the graph stopped and a fresh snapshot was emitted; the
  external client decides whether to call its model again.
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
model quality evaluation.
