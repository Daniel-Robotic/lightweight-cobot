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
