"""Manual bounded real-SDK/JTC integration check over loopback only.

Run after building/sourcing iiwa_controller; never connects physical hardware.
"""
import csv
import sys
import time
import os
import signal
import subprocess
import tempfile
from pathlib import Path
import xacro
import yaml

root = Path(__file__).resolve().parents[3]
work = Path(tempfile.mkdtemp(prefix='iiwa-fri-smoke-'))
env = dict(os.environ, ROS_DOMAIN_ID='83', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(work / 'logs'), ROS_HOME=str(work / 'ros'))
env['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
xml = xacro.process_file(str(root / 'src/iiwa_description/urdf/iiwa7.urdf.xacro'), mappings={'robot_ip': '127.0.0.2', 'fri_port': '30283', 'fri_cycle_ms': '10'}).toxml()
assert '<param name="simulate">false</param>' in xml
assert '<param name="robot_ip">127.0.0.2</param>' in xml
assert '<param name="fri_port">30283</param>' in xml
build = Path(sys.argv[1]) if len(sys.argv) > 1 else root / 'build/iiwa_controller'
server = build / 'fake_fri_server'
shim = build / 'libfri_loopback_bind.so'
assert server.is_file() and shim.is_file(), 'Build the test harness first'
trace = work / 'commands.csv'
params = work / 'robot.yaml'
params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': xml}}}))
processes = []
handles = []
try:
    server_log = (work / 'fake_fri_server.log').open('w')
    handles.append(server_log)
    processes.append(subprocess.Popen([str(server), str(trace)], stdout=server_log, stderr=subprocess.STDOUT, start_new_session=True))
    time.sleep(0.1)
    assert processes[0].poll() is None, 'Fake peer failed to bind'
    for package, executable, extra in [
        ('robot_state_publisher', 'robot_state_publisher', []),
        ('controller_manager', 'ros2_control_node', ['--params-file', str(root / 'src/iiwa_config/config/moveit/iiwa_controller.yaml'), '-p', 'update_rate:=100', '-p', 'hardware_synchronization.expect_blocking_read_write:=true']),
    ]:
        log = (work / (executable + '.log')).open('w')
        handles.append(log)
        processes.append(subprocess.Popen(['ros2', 'run', package, executable, '--ros-args', '--params-file', str(params)] + extra,
                                          env=dict(env, **({'LD_PRELOAD': str(shim)} if executable == 'ros2_control_node' else {})), stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    for name in ['joint_state_broadcaster', 'iiwa_arm_controller']:
        result = subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', name, '--controller-manager-timeout', '15'],
                                env=env, text=True, capture_output=True, timeout=25)
        print(result.stdout, result.stderr, flush=True)
        assert result.returncode == 0, name
    result = subprocess.run(['ros2', 'control', 'list_controllers'], env=env, text=True, capture_output=True, timeout=10)
    print(result.stdout, result.stderr, flush=True)
    assert result.returncode == 0 and 'iiwa_arm_controller' in result.stdout and 'active' in result.stdout
    initial = yaml.safe_load((root / 'src/iiwa_config/config/moveit/initial_positions.yaml').read_text())['initial_positions']
    positions = [float(initial[f'joint{i}']) for i in range(1, 8)]
    positions[0] += 0.02
    goal = {'trajectory': {'joint_names': [f'joint{i}' for i in range(1, 8)],
            'points': [{'positions': positions, 'time_from_start': {'sec': 2, 'nanosec': 0}}]}}
    result = subprocess.run(['ros2', 'action', 'send_goal', '/iiwa_arm_controller/follow_joint_trajectory',
                             'control_msgs/action/FollowJointTrajectory', yaml.safe_dump(goal)],
                            env=env, text=True, capture_output=True, timeout=15)
    print(result.stdout, result.stderr, flush=True)
    assert result.returncode == 0 and 'SUCCEEDED' in result.stdout
    rows = list(csv.DictReader(trace.open()))
    assert len(rows) > 100 and processes[0].poll() is None, 'FRI stream stopped'
    active = [r for r in rows if int(r['session']) == 4]
    assert active and abs(float(active[-1]['position']) - 0.02) < 0.001
    deltas = [abs(float(r['delta'])) for r in active]
    intervals = [float(b['receive_seconds']) - float(a['receive_seconds']) for a, b in zip(active, active[1:])]
    assert max(deltas) <= 1.71 * 0.01 + 1e-6, 'Command exceeds per-frame velocity limit'
    print(f'SMOKE PASS: real SDK loopback + actual CM/JTC action succeeded; {len(active)} active frames, max delta={max(deltas):.8f} rad; receive interval min/max={min(intervals):.6f}/{max(intervals):.6f} s.', flush=True)
finally:
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(process.pid, signal.SIGKILL)
    for handle in handles: handle.close()
    for log in work.glob('*.log'):
        print('\n' + log.name + '\n' + log.read_text()[-4500:])
    print('Logs:', work)
