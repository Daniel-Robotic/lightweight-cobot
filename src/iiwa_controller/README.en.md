# iiwa_controller

KUKA LBR iiwa 7 R800 `hardware_interface::SystemInterface`, targeting the bundled
**FRI C++ SDK 1.16**, ROS 2 Jazzy, **hardware_interface 4.48.0**, C++17.
[Русский](README.md).

## Sources

The SDK 1.16 Doxygen in the `kuka-fri` skill and Jazzy hardware documentation
were read before reviewing the implementation. Bundled SDK sources,
`ServerFriRos2.java`, the URDF and the full
[KUKA Sunrise.FRI 1.16 manual](../../doc/pdf/KUKA_SunriseFRI_116_en.pdf)
were then checked. Relevant manual sections: 2.1.4, 6.2.1–6.2.5 and 6.3.3.
Monitoring mirrors commanded position; COMMANDING_WAIT mirrors IPO.
Commanding states require continuous commands. Measured, commanded and IPO
positions are distinct quantities.

The plugin uses framework-managed interfaces and the installed Jazzy headers,
as described in [Writing a Hardware Component](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html).

## Contract

- Seven joints, `joint1`…`joint7`, in SDK A1…A7 order. Command: position [rad].
  States: encoder position [rad], finite-difference velocity [rad/s] calculated
  with robot timestamps, measured effort and estimated external torque [Nm].
- No position or velocity EMA. Commands retain a URDF velocity step limit
  (`max_velocity * FRI period`); this is a rate limit, not an EMA filter.
  Non-finite/out-of-position-range commands fault. Existing robot limits are
  unchanged and come from Jazzy `HardwareInfo::limits`.
- POSITION with JOINT overlay supports position, joint impedance and Cartesian
  impedance control used by `ServerFriRos2`. TORQUE/WRENCH are unsupported.
  Monitoring without an overlay remains supported.
- The only launched motion controller is `iiwa_arm_controller`, a
  JointTrajectoryController. Action: `/iiwa_arm_controller/follow_joint_trajectory`.
  Existing joint-state broadcaster QoS is retained; no new topics or TF frames.
- Hardware IP/port reach the URDF from `robot.ip`/`robot.port`; project defaults
  remain `192.170.10.2:30200`. Match `robot.fri_cycle_ms` to the Sunrise period.

One thread owns SDK `step()`. Cyclic exchange uses `try_lock`, so a busy mutex
never stalls either cycle. Readers retain a complete previous snapshot with a
freshness check; writers retry next cycle. Startup allows 15 seconds for the
first UDP packet. After that, a failed `step()` is terminal. Activation requires
fresh telemetry, at least MONITORING_READY, and GOOD/EXCELLENT quality.

Missing telemetry/commands for 100 ms, non-increasing timestamps, invalid
telemetry, unexpected commanding mode, a safety stop, inactive drives or poor
commanding quality stop command generation and surface ERROR to ROS. Leaving
COMMANDING_ACTIVE requires explicit lifecycle recovery and a new Sunrise
session. The 15 s/100 ms thresholds, command guards and explicit-recovery policy
are driver decisions, not universal KUKA requirements. Thread joining precedes
socket closure. Cleanup, error, shutdown and destruction release application,
connection, then client. Stopping FRI does not replace hardware emergency stop.

Dependencies remain those in package.xml; SDK sources build as `fri_client_sdk`.
Relevant headers are `friLBRClient.h`, `friClientApplication.h`,
`friUdpConnection.h`, and `friException.h`. SDK `FRIException` is caught explicitly.
Bundled SDK files are unchanged.

## Offline validation

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select iiwa_controller iiwa_bringup iiwa_config iiwa_description iiwa_utils --symlink-install
source install/setup.bash
colcon test --base-paths src --packages-select iiwa_controller
colcon test-result --test-result-base build/iiwa_controller --verbose
/usr/bin/python3 src/iiwa_controller/test/smoke_jtc.py
```

Tests exercise real SDK callbacks with artificial data and the plugin's local
`simulate=true` path: measurements, IPO, no EMA, modes, limits, faults and
repeated lifecycle transitions. They do not establish physical FRI timing.

Before hardware use, run `cobot run local`, choose **Webots simulator**, verify
that the broadcaster and JTC are active, execute and cancel a small valid
MoveIt trajectory, and check the action result and simulated stop. Webots uses
a separate hardware plugin; it does not validate UDP/FRI. No physical robot was
run for this revision. Hardware testing requires the networking, limits,
tool/load, mode, operator supervision and stop checks in AGENTS.md.

Revision results: all five affected ROS packages built; 14 C++ cases in two
CTest checks passed. The local smoke script loaded the actual URDF with our
plugin in simulation mode, activated JSB/JTC and completed a +0.02 rad joint-1
trajectory with SUCCEEDED. On SIGINT, Controller Manager's `pal_statistics`
reported an invalid shutdown context; hardware lifecycle shutdown succeeded.
Webots and physical FRI were not run.
