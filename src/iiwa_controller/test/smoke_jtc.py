"""Manual local JTC integration check; never opens a FRI connection."""
import os
import signal
import subprocess
import tempfile
from pathlib import Path
import xacro
import yaml

root = Path(__file__).resolve().parents[3]
work = Path(tempfile.mkdtemp(prefix='iiwa-controller-smoke-'))
env = dict(os.environ, ROS_DOMAIN_ID='83', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(work / 'logs'), ROS_HOME=str(work / 'ros'))
env['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
xml = xacro.process_file(str(root / 'src/iiwa_description/urdf/iiwa7.urdf.xacro')).toxml()
assert '<param name="simulate">false</param>' in xml
# Exercise our plugin's explicit local simulation path, with no FRI socket.
xml = xml.replace('<param name="simulate">false</param>', '<param name="simulate">true</param>')
params = work / 'robot.yaml'
params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': xml}}}))
processes = []
handles = []
try:
    for package, executable, extra in [
        ('robot_state_publisher', 'robot_state_publisher', []),
        ('controller_manager', 'ros2_control_node', ['--params-file', str(root / 'src/iiwa_config/config/moveit/iiwa_controller.yaml')]),
    ]:
        log = (work / (executable + '.log')).open('w')
        handles.append(log)
        processes.append(subprocess.Popen(['ros2', 'run', package, executable, '--ros-args', '--params-file', str(params)] + extra,
                                          env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
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
    print('SMOKE PASS: actual URDF, iiwa hardware plugin, broadcaster and JTC active; no FRI connection.', flush=True)
finally:
    for process in processes:
        os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(process.pid, signal.SIGKILL)
    for handle in handles: handle.close()
    for log in work.glob('*.log'):
        print('\n' + log.name + '\n' + log.read_text()[-4500:])
    print('Logs:', work)
