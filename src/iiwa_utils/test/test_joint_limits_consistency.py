from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml
import xacro
import xacro.substitution_args

SRC = Path(__file__).resolve().parents[2]
LIMITS = SRC / 'iiwa_config/config/moveit/joint_limits.yaml'


def test_all_seven_axes_share_limits(monkeypatch):
    monkeypatch.setattr(xacro.substitution_args, '_eval_find', lambda p: str(SRC / p))
    data = yaml.safe_load(LIMITS.read_text())['joint_limits']
    for simulate in ('false', 'true'):
        robot = ET.fromstring(xacro.process_file(
            str(SRC / 'iiwa_description/urdf/iiwa7.urdf.xacro'),
            mappings={'simulate': simulate, 'joint_limits_file': str(LIMITS)}).toxml())
        for i, (degrees, speed) in enumerate(zip([170,120,170,120,170,120,175], [98,98,100,130,140,180,180]), 1):
            name = f'joint{i}'
            limit = data[name]
            physical = robot.find(f"joint[@name='{name}']/limit")
            assert float(physical.attrib['lower']) == limit['min_position']
            assert float(physical.attrib['upper']) == limit['max_position']
            assert float(physical.attrib['velocity']) == limit['max_velocity']
            assert limit['max_position'] <= math.radians(degrees) + 1e-12
            assert limit['min_position'] >= -math.radians(degrees) - 1e-12
            assert limit['max_velocity'] <= math.radians(speed) + 1e-12
            safety = robot.find(f"joint[@name='{name}']/safety_controller")
            assert float(safety.attrib['soft_lower_limit']) == limit['soft_min_position']
            assert float(safety.attrib['soft_upper_limit']) == limit['soft_max_position']
            command = robot.find(f"ros2_control/joint[@name='{name}']/command_interface")
            assert float(command.find("param[@name='min']").text) == limit['soft_min_position']
            assert float(command.find("param[@name='max']").text) == limit['soft_max_position']
            assert limit['max_acceleration'] > 0 and limit['max_jerk'] > 0


def test_planning_limits_keep_soft_margin():
    from iiwa_utils.converter import effective_joint_limits
    raw = yaml.safe_load(LIMITS.read_text())
    effective = effective_joint_limits(raw)
    for name, joint in raw['joint_limits'].items():
        assert effective['joint_limits'][name]['min_position'] == joint['soft_min_position']
        assert effective['joint_limits'][name]['max_position'] == joint['soft_max_position']
        assert joint['min_position'] < joint['soft_min_position']  # input not mutated
