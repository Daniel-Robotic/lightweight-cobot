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
