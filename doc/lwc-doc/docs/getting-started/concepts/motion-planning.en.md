# Motion planning

!!! info "Work in progress"
    A detailed description of motion planning is being prepared.

LWC uses **MoveIt 2**, the standard motion-planning framework for ROS 2.

## Key concepts

- **Planning group** (`iiwa_arm`) — the set of joints for which a plan is generated. Defined in SRDF.
- **Planner** — the trajectory-generation algorithm. Available planners:
  - `ompl` — general-purpose probabilistic planner (default)
  - `pilz_industrial_motion_planner` — deterministic PTP, LIN, and CIRC trajectories
- **TCP (Tool Center Point)** — the tool point for which the target pose is specified. Set in `cobot-setting.yaml` → `planning.pose_link`.
- **Reference frame** — the coordinate system for targets. Default: `base_link`.

## `cobot-setting.yaml` settings

| Parameter | Description |
|---|---|
| `planning.default_planner` | Default planner: `ompl` or `pilz_industrial_motion_planner` |
| `planning.planning_attempts` | Number of attempts after a planning failure |
| `planning.pose_link` | TCP link for Cartesian targets |
