# Simulation (Webots)

**Webots** is an open-source robot simulator. LWC uses it as a digital twin of the KUKA LBR IIWA 7, allowing control algorithms to be developed and debugged without a physical robot.

## Key features

- The simulator uses the same ROS 2 topics and interfaces as the real robot.
- The simulation world is set in `cobot-setting.yaml` → `digital_twin.webots.world`.
- Start it with `cobot run --simulate`.

## TCP gizmo

This simulation-only mode lets you specify the robot's target pose by dragging a marker at its tool center point (TCP) in the Webots window.

1. Enable `digital_twin.tcp_gizmo.enabled: true` in `cobot-setting.yaml` and launch with `cobot run --simulate`.
2. Wait until other robot motions finish. For an initial test, use the regular named-pose command to move to `work`.
3. Select the TCP marker in Webots. Drag a translation arrow or rotation handle with the left mouse button. The robot remains stationary while you edit the target.
4. Release the button to submit the target. One complete trajectory is sent to the controller, rather than a stream of actions during dragging.
5. Wait for completion before dragging again. If the target cannot be reached or the checked path collides, the marker returns to the actual TCP.

Regular coordinate, joint, and control-file commands remain available. They have priority over new gizmo gestures. An already started gizmo trajectory finishes before a waiting regular command proceeds, even when that wait exceeds two seconds.

### Tool attachment and speed

The marker follows the active tool's `tcp` frame. With `patron`, it is at the chuck's working end; with no tool, it is at the flange. Apply tool changes through `cobot robot-setup`, rebuild, and restart the simulation. New tool models must define `tcp` at their working point.

`max_linear_speed` (m/s) and `max_angular_speed` (rad/s) are upper bounds. Short moves include acceleration and braking, and joint limits may make motion slower. Joint interpolation reaches the requested TCP pose but does not guarantee a straight Cartesian path.

Set `digital_twin.tcp_gizmo.enabled: false` and restart to disable this mode. The [configuration reference](../configuration.md) describes all parameters. `command_timeout` checks the age of the submitted target; mouse release is detected directly and requires no waiting-time setting.

### When the marker returns or does not respond

- Wait for regular motions or a committed gizmo trajectory to finish before starting another drag.
- For a rejected target, read `TCP gizmo rejected target: ...` in the launch output. Try a smaller displacement from a reachable posture such as `work`.
- With missing or stale robot feedback, the gizmo cannot submit movement. Check that simulation is running and `/joint_states` updates.

## Differences from the real robot

- Motion still obeys configured joint and TCP limits; simulation does not reproduce all physical robot protections.
- Physics is approximate, including inertia, friction, and elasticity.
- FRI is not used; communication goes through the Webots ROS 2 driver.

## Spawning textured objects

Set `digital_twin.webots.objects` in `cobot-setting.yaml`:

```yaml
objects:
  enabled: true
  count: 5
  models:
    - path: pkg://iiwa_description/objects/hammer/hammer1.obj
      rotation_deg: [0, 0, 0]
    - path: pkg://iiwa_description/objects/hammer/hammer2.obj
      rotation_deg: [0, 0, 0]
  area:
    x: [0.3, 0.7]
    y: [-0.3, 0.3]
    z: [0.1, 0.1]
```

Webots spawns exactly `count` instances once at startup, independently choosing
from `models` with replacement. This list specifies allowed models, not individual
quantities. An absent section, `enabled: false`, or `count: 0` disables spawning.
Paths support `pkg://`, absolute paths and paths relative to the settings file.
Currently only Wavefront OBJ is supported.

Bounds are in metres in the robot base frame. The spawner uses the same
`webots.transform` and `webots.rotation` (axis and angle in radians) as the robot:
`p_world = t_robot + R_robot * p_local`. They constrain initial spawn positions;
physics may move the objects afterwards. Moving the robot with the mouse after
startup does not relocate existing objects.

The spawn anchor is the bounding-box XY centre and bottom Z. Exported vertex
offsets are compensated without modifying the original files, UVs, scale
(1 unit = 1 metre); orientation is configured per model. Bounds constrain the anchor, not the entire
object. Random placements may overlap; choose a sufficiently large area.

`CadShape` loads OBJ materials through `mtllib` and textures through MTL. Preserve
relative asset paths: for example, `hammer1.mtl` references
`hammer1_basecolor.png`. The complete `objects` directory is installed with
`iiwa_description`. Missing models, materials or textures fail before the first
spawn request. Physics uses an approximate bounding Box and a mass of 0.5 kg,
not calibrated hammer inertia or grasp geometry. Objects are not automatically
added to the MoveIt Planning Scene.

The `object_spawner` node calls `/Ros2Supervisor/spawn_node_from_string`
(`webots_ros2_msgs/srv/SpawnNodeFromString`) once per object. A rejected request
aborts remaining work. Timeouts are not retried to avoid duplicates. The node
exits after success and only runs in simulation.

### Simulation smoke test

1. Build with `colcon build --base-paths src --packages-select iiwa_description iiwa_utils iiwa_bringup iiwa_config --symlink-install`.
2. Run `source install/setup.bash`, then `cobot run local`, selecting Webots.
3. Inspect the five nodes `spawned_object_0000` through `spawned_object_0004`, their
   textures and the `All 5 objects spawned` log.
4. Change `transform` and `rotation`, restart and confirm the spawn area translates
   and rotates with the robot base. Pause before objects fall to inspect initial positions.
5. Check `count: 0`, operation without cameras and rejection of missing models.

Reference: [Webots CadShape](https://cyberbotics.com/doc/reference/cadshape).

### Per-model rotation

Each `models` entry accepts `path` and `rotation_deg: [X, Y, Z]`.
Angles are fixed degrees: apply X, then Y, then Z about the robot base axes,
`R_model = Rz * Ry * Rx` and `R_world = R_robot * R_model`. For example:

```yaml
models:
  - path: pkg://iiwa_description/objects/hammer/hammer1.obj
    rotation_deg: [0, 0, 0]
  - path: pkg://iiwa_description/objects/hammer/hammer2.obj
    rotation_deg: [90, 0, 0]
  - path: pkg://iiwa_description/objects/hammer/hammer3.obj
    rotation_deg: [0, 0, 45]
```

The supplied OBJ hammers have their long axis along Z. `[0, 0, 0]` gives an
initial vertical orientation, `[90, 0, 0]` lays the hammer sideways and
`[0, 0, 45]` turns it around the vertical axis. Both configuration files now
explicitly set `[0, 0, 0]` for each model; adjust each independently.
Legacy path-only entries remain supported and imply zero rotation.

Both the visible model and its collision Box rotate. Height is recalculated:
`area.z` locates the bottom of the rotated collision bounds in the robot frame;
X/Y locate their centre. Model rotation does not rotate the spawn area.
Changes take effect on the next launch.

This sets the initial orientation, not a constraint. Gravity and contact can
still topple the hammer. Pause simulation at spawn to inspect it. Smoke-test
two models with different angles, check the Box stays above `area.z`, and check
that changing `webots.rotation` preserves each model's relative orientation.
