# Proposal

## Why

The user trains and runs Qwen outside ROS 2, while the robot project already exposes a live scene and accepts plans over HTTP. The missing contract is a reliable way to give the model the current scene, skill catalog, system prompt and graph schema, then submit its `graph-v3` answer and observe the run; the current executor accepts a different graph format and cannot safely consume the trained model output directly.

## What Changes

- Add a planner context API that returns a fresh, versioned model input package: canonical system prompt, live scene, available skills/catalog, graph-v3 JSON Schema, contract versions and state version.
- Add explicit graph-v3 submission support for `ready`, `blocked` and `clarification_required`; only validated `ready` plans may enter execution. The service must reject unsupported or unsafe graph constructs before creating a run record.
- Validate graph references and node/event structure, available skills, object/tool/location IDs and scene freshness against the authoritative runtime snapshot, then adapt supported graph-v3 actions to the existing execution engine.
- Keep model inference external: ROS 2 does not load model weights, call a provider or retain model credentials.
- Use graph-v3 as the only accepted planner response format; reject the old executor graph format instead of maintaining a legacy path. Preserve run status and cancellation behavior for graph-v3. Document an end-to-end Webots workflow and the physical-robot boundary when implemented.

## Capabilities

### New Capabilities

- `llm-plan-execution`: Accept and safely execute compatible external graph-v3 plans against a live ROS scene.

### Modified Capabilities

- `external-api`: Add a model-context endpoint and graph-v3 submission behavior to the existing planner HTTP API.

## Impact

- Affects `src/iiwa_web/iiwa_web/planner.py`, the graph parser/validator and execution boundary in `src/iiwa_llm_plan_executor/`, the live scene/catalog contract in `src/iiwa_scene_context/`, ROS interfaces if required, and their tests.
- The dataset project and its trained model remain external and unchanged. The API returns information; the operator/model client supplies the natural-language task and model response.
- The trained report shows 100% graph-v3 schema validity, but `status_acc=0.57`; in the washer-kit test all 400 blocked examples were incorrectly called ready, and in defect-quarantine all 400 ready examples were called blocked. Therefore server-side runtime validation, not model status claims, determines whether a plan can execute.
- Runtime scene schema v2 and dataset scene schema v1 are not wire-compatible today. The change must define and test the scene/catalog conversion boundary. Executor graph format v2 is not a supported input target after this change; graph-v3 is the sole public and execution contract.
