import yaml
import copy
import math
import tempfile

from pathlib import Path
from typing import Dict, Optional, Union

import xacro


def load_robot_description(
    model_path: Union[str, Path],
    robot_name: str,
    xacro_args: Optional[Dict[str, str]] = None,
) -> str:
    model_path = Path(model_path)
    suffix = model_path.suffix.lower()

    if suffix == ".xacro":
        mappings = {"name": str(robot_name)}
        if xacro_args:
            mappings.update({k: str(v) for k, v in xacro_args.items()})

        return xacro.process_file(str(model_path), mappings=mappings).toxml()

    if suffix == ".urdf":
        return model_path.read_text(encoding="utf-8")

    raise FileNotFoundError(f"Supported file formats: .xacro/.urdf, got: {model_path}")


def effective_joint_limits(data: dict) -> dict:
    """Keep MoveIt's YAML overrides inside the same protective bounds as FRI."""
    result = copy.deepcopy(data)
    for name, joint in result['joint_limits'].items():
        joint['min_position'] = max(joint['min_position'], joint.get('soft_min_position', joint['min_position']))
        joint['max_position'] = min(joint['max_position'], joint.get('soft_max_position', joint['max_position']))
        if not all(math.isfinite(joint[key]) for key in ('min_position', 'max_position', 'max_velocity', 'max_acceleration', 'max_jerk')):
            raise ValueError(f'Non-finite joint limits: {name}')
        if joint['min_position'] >= joint['max_position'] or any(joint[key] <= 0 for key in ('max_velocity', 'max_acceleration', 'max_jerk')):
            raise ValueError(f'Invalid joint limits: {name}')
    return result


def wrap_for_ros2_params(yaml_path: str, namespace: str) -> str:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    if namespace == 'robot_description_planning':
        data = effective_joint_limits(data)
    wrapped = {namespace: {"ros__parameters": data}}
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(wrapped, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name