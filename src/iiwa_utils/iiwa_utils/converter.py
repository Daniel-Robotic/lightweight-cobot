import yaml
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


def wrap_for_ros2_params(yaml_path: str, namespace: str) -> str:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    wrapped = {namespace: {"ros__parameters": data}}
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(wrapped, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name