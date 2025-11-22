import xacro
from pathlib import Path


def load_robot_description(model_path: Path, robot_name: str) -> str:
    suffix = Path(model_path).suffix.lower()
    if suffix == ".xacro":
        return xacro.process_file(model_path, mappings={'name': str(robot_name)}).toxml()
    elif suffix == ".urdf":
        return Path(model_path).read_text(encoding="utf-8")
    else:
        raise FileNotFoundError(f"Supported file formats: xacro/urdf")