# System configuration

## Main configuration file

All system parameters are stored in a single file: **`cobot-setting.yaml`** in the project root. It is the single source of truth for the robot IP address, ports, configuration paths, planner settings, and web server settings.

!!! info "No manual configuration is needed before installation"
    When you run `cobot setup`, the wizard offers to configure this file automatically in **step 2**. Return to this section when you want to change parameters after the initial installation.

!!! danger "Do not edit the file manually"
    Use only `cobot robot-setup`. The interactive wizard validates values and prevents syntax errors. Editing the YAML manually may cause parsing errors and prevent the system from starting.

```bash
cobot robot-setup
```

---

## `robot` section — robot parameters

Controls the connection to the physical KUKA controller through FRI.

```yaml
robot:
  name: "iiwa7"
  ip: "192.170.10.2"
  port: 30200
  fri_cycle_ms: 10
  description: pkg://iiwa_description/urdf/iiwa7.urdf.xacro
```

| Parameter | Description | Recommendation |
|---|---|---|
| `name` | Robot model | Do not change: `iiwa7` |
| `ip` | KUKA controller IP address | **Change** to the actual controller address |
| `port` | FRI UDP port | Default: `30200`; change only if the port conflicts |
| `fri_cycle_ms` | FRI cycle: `5` ms = 200 Hz, `10` ms = 100 Hz | Use `10` for stable operation or `5` for high-precision tasks |
| `joint_position_tau` | FRI command EMA time constant, s | `0.04`; `0` disables smoothing |
| `description` | Path to the robot URDF | Do not change |

Motion always uses JointTrajectoryController. The position command is smoothed before FRI transmission with `robot.joint_position_tau`; no velocity filter is applied.

---

## `digital_twin` section — simulator

Configures the Webots environment and RViz visualization.

```yaml
digital_twin:
  tcp_gizmo:
    enabled: true
    publish_period: 0.032
    command_timeout: 0.3
    state_timeout: 0.5
    max_linear_speed: 0.50
    max_angular_speed: 0.4
  webots:
    world: pkg://iiwa_description/worlds/iiwa.wbt
    transform: "-0.25 0 0.79"
    rotation: "0 0 1 0"
    controller_timer: "50"
    cameras:
      - pkg://iiwa_config/config/cameras/d455_top.yaml
  rviz:
    config: pkg://iiwa_config/config/rviz/rviz_moveit.rviz
```

| Parameter | Description |
|---|---|
| `webots.world` | Path to the simulator `.wbt` world |
| `webots.transform` | Robot base offset in the world `[x y z]`, in meters |
| `webots.rotation` | Base orientation `[x y z angle]`, in radians |
| `webots.cameras` | List of YAML configurations for connected cameras |
| `rviz.config` | Path to the RViz configuration |

### TCP gizmo in Webots

Select the TCP marker, translate or rotate it with the native Webots gizmo, and release the left mouse button. Holding the button only changes the preview. Releasing it submits one complete trajectory. Another drag is accepted after that trajectory finishes. An unreachable target or a colliding trajectory resets the marker to the current TCP.

The gizmo controls the active tool's `tcp` link independently of `planning.pose_link`. For `patron`, TCP is at the chuck's working end, 55.2 mm along local Z from the `patron` link origin. Without a tool, it coincides with the `link_ee` flange. Regular Cartesian commands also use this point when `planning.pose_link: tcp`.

The gizmo has the lowest motion priority. It is unavailable during regular motion. If a coordinate, joint, or file command arrives while a committed gizmo trajectory is running, that command waits for the trajectory to finish before proceeding.

| `tcp_gizmo` parameter | Description |
|---|---|
| `enabled` | Set to `false` to disable the gizmo at the next launch; simulation only |
| `publish_period` | State update period, seconds |
| `command_timeout` | Maximum age of an incoming target, seconds; not a delay after dragging |
| `state_timeout` | Maximum age of robot feedback, seconds |
| `max_linear_speed` | TCP linear speed cap, m/s |
| `max_angular_speed` | TCP angular speed cap, rad/s |

Mouse release is detected directly, so no idle timer is needed. Joint limits, acceleration, and deceleration can reduce speed below the configured cap. The trajectory interpolates joint positions to reach the requested TCP pose; a straight Cartesian path is not guaranteed.

See the [simulation guide](concepts/simulation.md) for the operating sequence.



---

## `tool` section — active tool

Specifies which gripper or tool is attached to the robot.

```yaml
tool:
  active: "patron"
```

| Value | Description |
|---|---|
| `none` | No tool |
| `patron` | Patron chuck/gripper |

Available tools are defined in `src/iiwa_config/config/tools.yaml`. To add a tool, describe it there and then set its name in `tool.active`.

Apply tool selection through `cobot robot-setup`, which updates the tool description and SRDF. Rebuild the project and restart the simulation after changing tools.

---

## `planning` section — motion planning

Configures MoveIt 2 and the trajectory planner.

```yaml
planning:
  pose_link: "tcp"
  planning_group: "iiwa_arm"
  default_frame: "base_link"
  default_planner: "ompl"
  planning_attempts: 3
```

| Parameter | Description | Recommendation |
|---|---|---|
| `pose_link` | TCP link used for Cartesian targets | Must match the URDF frame; do not change without updating the URDF |
| `planning_group` | Planning group from the SRDF | Do not change: `iiwa_arm` |
| `default_frame` | Default reference frame | Do not change: `base_link` |
| `default_planner` | Planner: `ompl` or `pilz_industrial_motion_planner` | `ompl` is general purpose; `pilz` produces predictable trajectories |
| `planning_attempts` | Number of planning attempts after failure | Increase for difficult trajectories |

---

## `web` section — REST API and MCP server

Configures the FastAPI server used to control the robot over HTTP and MCP for AI-agent integration.

```yaml
web:
  enabled: true
  host: "0.0.0.0"
  port: 8007
  endpoints: pkg://iiwa_config/config/api_endpoints.yaml
  joint_limits: pkg://iiwa_config/config/moveit/joint_limits.yaml
```

| Parameter | Description |
|---|---|
| `enabled` | Enable (`true`) or disable (`false`) the web server |
| `host` | Listening address: `0.0.0.0` for all interfaces or `127.0.0.1` for local access only |
| `port` | HTTP API port; default: `8007` |
| `endpoints` | Path to the REST endpoint description |
| `joint_limits` | Path to joint limits used for command validation |

After startup, the REST API is available at `http://<host>:8007`, and MCP is available at `/mcp`.

---

## `foxglove` section — Foxglove Studio monitoring

[Foxglove Studio](https://foxglove.dev/) visualizes and monitors ROS 2 topics in real time.

```yaml
foxglove:
  enabled: true
  port: 8765
  debug: false
  address: 0.0.0.0
```

| Parameter | Description |
|---|---|
| `enabled` | Enable or disable Foxglove Bridge |
| `port` | WebSocket port used by Foxglove Studio; default: `8765` |
| `debug` | Detailed logging for the bridge process |
| `address` | WebSocket listening address |

The remaining parameters (`tls`, `topic_whitelist`, `min_qos_depth`, and others) are intended for advanced configuration and normally do not need to be changed.

---

## What to change and what to keep

| | Parameter | Action |
|---|---|---|
| ✅ | `robot.ip` | **Must be changed** to the controller IP address |
| ✅ | `robot.fri_cycle_ms` | Select `10` (standard) or `5` (high frequency) |
| ✅ | `tool.active` | Set the active tool |
| ✅ | `web.enabled` | Set to `false` if the web interface is not needed |
| ⚠️ | `planning.*` | Change only when another planner or other parameters are required |
| ❌ | `robot.description` | Do not change; this is the URDF path |
| ❌ | `controller.moveit.*` | Do not change; these are package-internal MoveIt configuration paths |
| ❌ | `digital_twin.webots.world` | Do not change unless you understand the Webots world structure |
