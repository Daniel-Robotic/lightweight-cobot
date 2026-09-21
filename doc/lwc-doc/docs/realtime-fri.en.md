# FIFO priority for FRI

## Why it is needed

FRI exchanges a position command on every cycle. With a 10 ms period, a new packet should be handled about every 10 ms. The regular Linux scheduler can delay the control thread behind other processes. The log then contains messages such as:

```text
Could not enable FIFO RT scheduling: Operation not permitted
Overrun might occur ... Read time: 11500 us
```

`SCHED_FIFO` is Linux scheduling for threads with fixed priority. This project requests priority 50 for `controller_manager` and priority 80 for the FRI worker. The FRI worker receives the higher priority so it can receive a KUKA packet and send the reply on time.

Configure this on the **ROS 2 computer** that runs `cobot run`. It does not require changes to the Sunrise Cabinet or `cobot-setting.yaml`.

## Ubuntu 24.04 setup

On the ROS 2 computer, run these commands as the user who starts `cobot`:

```bash
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"
sudo tee /etc/security/limits.d/99-ros2-realtime.conf >/dev/null <<'EOF'
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
EOF
```

Log out of the desktop session completely and log in again, or reboot. An already open terminal does not receive the new group and limits.

## Verification

In a new terminal, verify the group and limits:

```bash
id -nG | tr ' ' '\n' | grep -x realtime
ulimit -r
ulimit -l
```

The expected output is `realtime`, then `99`, then `unlimited`.

Start FRI without commanding robot motion and check the log for:

```text
FRI worker uses FIFO priority 80
```

You can also inspect the policies of all threads in the process:

```bash
PID=$(pgrep -n ros2_control_node)
ps -L -p "$PID" -o pid,tid,cls,rtprio,pri,comm
```

Realtime threads have `FF` in the `CLS` column. The FRI worker has `RTPRIO` 80 and the `controller_manager` main loop has 50.

!!! warning "Before the first motion"
    First confirm that FRI connects and that the log contains `FRI worker uses FIFO priority 80`. Then make a short move at speed 0.1 with an operator supervising. FIFO reduces jitter on the computer; it does not replace network checks, robot limits, or a clear work area.

## If setup does not work

- If `realtime` is missing from `id`, log out and log in again.
- If `ulimit -r` is less than 80, check `/etc/security/limits.d/99-ros2-realtime.conf` and that PAM loads `pam_limits.so`.
- If the log still says `Operation not permitted`, make sure `cobot run` is started by the same user passed to `usermod`.
- A Python virtualenv needs no extra setup: permissions are assigned to the user and inherited by Python.
- Docker requires `--cap-add=sys_nice --ulimit rtprio=99 --ulimit memlock=-1`.

## Rollback

To remove the setup, run:

```bash
sudo rm /etc/security/limits.d/99-ros2-realtime.conf
sudo gpasswd -d "$USER" realtime
```

After logging in again, processes use the regular Linux scheduler.

See the official [ros2_control documentation](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html#determinism) for controller-manager priorities and limits.

## Trajectory limits and smoothness

`controller.moveit.joint_limits` in `cobot-setting.yaml` selects a single limits file for all seven axes, consumed by xacro, FRI, Webots, MoveIt and the web API. Hard position limits and the existing narrower protective bounds remain distinct. Command validation and planning use their intersection; the MoveIt parameter override explicitly preserves soft bounds.

Position and velocity limits were checked against the [KUKA LBR iiwa 7 R800 datasheet, 0000-246-832 V1.2](https://www.kuka.com/-/media/kuka-downloads/imported/8350ff3ca11642998dbdc81dcc2ed44c/0000246832_en.pdf). Working limits were not expanded relative to the previous URDF. Upward rounding beyond published values was removed. **Acceleration and jerk limits remain project settings, not verified KUKA ratings.** Tool/payload and Sunrise settings still require independent verification.

OMPL/CHOMP now run TOTG → rest-boundary normalization → Ruckig → rest-boundary normalization → `iiwa_planning/LimitSplineDynamics` → MoveIt solution validation. The initial requested state must be stationary within `planning.stopped_velocity_tolerance` (rad/s). The spline adapter bounds the complete JTC quintic polynomial in Bernstein form, including extrema between waypoints. It uniformly stretches time and scales derivatives to respect velocity, acceleration and jerk limits, rejecting position overshoot. Endpoint acceleration is normalized around Ruckig because TOTG and Ruckig duration extension can leave nonzero boundary derivatives.

Pilz PTP/LIN/CIRC use the spline validator and uniform time scaling without joint-space Ruckig. This preserves the original JTC interpolation curve, not a new guarantee of ideal Cartesian interpolation. Nonstationary boundary conditions, including some blended sequences, are rejected; this implementation targets rest-to-rest motions.

Position-only CSV/JSON `/trajectory/send*` requests are fitted with a smooth rest-to-rest spline and passed through the same C++ validator. Input waypoints are retained; the curve between them can differ from the former linear interpolation. This direct API does not check collisions. Responses include actual `duration` and `time_scale`; timing may increase. `validate_limits=false` cannot disable mandatory dynamics and operating-range checks.

Direct commands require fresh, complete named joint position/velocity feedback and a stationary robot. The first point at zero time must match the measured position within `web.trajectory_start_tolerance`; otherwise a measured start point is prepended if the first timestamp is positive. Freshness and stationarity are checked again after preparation. `web.trajectory_state_timeout` is in seconds; `web.trajectory_stopped_velocity` is in rad/s. Concurrent direct commands are rejected while a trajectory is reserved; stopping cancels pending preparation.

External clients sending directly to the JTC topic/action bypass this MoveIt/web preparation and remain responsible for their trajectories. This is not a global ROS command firewall. The FRI filter, watchdog and cycle period are unchanged. Hardware retesting is needed to assess noise and actual tracking.

After rebuilding, run `colcon test --packages-select iiwa_planning iiwa_utils iiwa_web iiwa_controller --python-testing pytest`. Tests exercise seven-axis consistency, spline extrema, the installed JTC interpolator, installed TOTG/Ruckig plus pluginlib, and web admission failures without connecting to a real FRI session. Rebuild `iiwa_config`, `iiwa_description` and `iiwa_bringup` as well; running processes do not update automatically.

The adapters check original `/joint_states` telemetry: absent MoveIt `RobotState` velocities are not treated as zero. `planning.state_timeout` bounds telemetry age. The subscription test uses synthetic messages in separate local ROS domain 231, without FRI.
