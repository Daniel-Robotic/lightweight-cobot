# `iiwa_scene_context` interfaces

Contract version: **1**. Catalog version: **1**. Scene schema version: **1**.
All JSON is UTF-8 and uses stable Webots object ids.

## ROS interfaces

| Direction | Name | Type | Notes |
| --- | --- | --- | --- |
| service | `/iiwa_scene_context/get_scene` | `std_srvs/Trigger` | `success=true` and `message` = snapshot envelope JSON, or `success=false` and a reason (not ready / stale / no state). |
| topic | `/iiwa_scene_context/scene_json` | `std_msgs/String` | The same envelope JSON. QoS: reliable, transient-local, depth 1. A late subscriber receives the latest snapshot. |
| input | `/iiwa_experement/object_state` | `std_msgs/String` | Live supervisor JSON. Only logical ownership and object state are used. |
| input | `/iiwa_experement/scene_ready` | `std_srvs/Trigger` | Polled by the node. Until it returns `success=true`, no snapshot is published. |

The node refuses with `success=false` while the scene is not ready, before the
first supervisor state, or when the last state is older than
`freshness_sec` (default 2.0 s).  A stale snapshot must never be treated as
`found`/`success` by a consumer.

## Snapshot envelope

```json
{
  "schema_version": 1,
  "contract_version": 1,
  "catalog_version": 1,
  "experiment_id": 20,
  "state_version": 7,
  "scene_revision": 3,
  "timestamp": 12.5,
  "scene_json": { "...": "see below" }
}
```

* `state_version` is the supervisor's discrete `version`.
* `scene_revision` starts at 1 and grows **only** when `state_version`
  changes. Pose updates of a held object published between ownership changes
  do not advance it. A graph built for another revision is rejected.
* `timestamp` is the local receive time of the snapshot. Consumers apply
  their own freshness threshold.

## `scene_json`

Fields: `schema_version`, `contract_version`, `catalog_version`, `profile`,
`experiment_id`, `domain`, `components`, `destinations`, `tools`,
`initial_state`, `cameras`, `available_skills`, `skill_contracts`,
`service_objects`, `recovery_attempts_per_component`.

`components` contains only objects present in the selected experiment
(`fixture`, `body`, and conditionally `cap`, `tray`, `screw_1`…`screw_10`).
`destinations` contains `work`, `hatch_seat`, `hole_1`…`hole_4`, one
`<tool>_socket` for every present tool, and the configured sealant sides.
`tools` contains only present tools; every tool carries its own socket id.

`initial_state` is logical only:

```json
{
  "tool_mounted": "vacuum",
  "held_object": "cap",
  "flags": ["tool_mounted:vacuum", "holding:cap", "hatch_held"],
  "components": {"cap": "held", "screw_1": "tray"},
  "tools": {"vacuum": "docked", "magnetic_tool": "docked"},
  "holes": {"hole_1": {"state": "empty", "screw": null}},
  "scene_ready": true
}
```

No coordinates, rotations, TF frames, supervisor poses or other supervisor
internals are part of `scene_json`. Exact object poses are read from the
supervisor by skill execution and never by the model.

`available_skills` and `skill_contracts` come from the server catalog in
`iiwa_scene_context/catalog.py`; a graph cannot extend them. `tighten` is not
available in this stage.
