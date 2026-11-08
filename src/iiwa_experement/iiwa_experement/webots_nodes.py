"""Turn a validated ScenarioSpec into deterministic Webots ``Solid`` nodes.

The generated nodes are sent one at a time to
``/Ros2Supervisor/spawn_node_from_string``.  Their order is:

    fixture -> body -> rack -> working tools -> tray -> screws -> cap

All positions are the initial scene only.  The fixture, body, rack and tray are
fixed.  A loose hatch has physics; an installed hatch is locked.  Screws and
tools are static in their tray/socket: this package does not implement grasp,
tool docking or dynamic hatch fixation, so a later skill executor must release
them before a robot can pick them up.
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

from .scenario import ScenarioSpec


ASSETS = {
    'fixture': 'stapel.obj',
    'body': 'base-detail.obj',
    'rack': 'tool-magizine.obj',
    'tray': 'lotok.obj',
    'screw': 'vint.obj',
    'cap': 'cap.obj',
    'vacuum': 'vacum-tool.obj',
    'magnetic_tool': 'magnit-tool.obj',
    'sealant_tool': 'germetic-tool.obj',
    'torque_tool': 'wranch-tool.obj',
}

TOOL_NODE_NAMES = {
    'vacuum': 'grant_vacuum',
    'magnetic_tool': 'grant_magnetic_tool',
    'sealant_tool': 'grant_sealant_tool',
    'torque_tool': 'grant_torque_tool',
}

TOOL_SIZE = {
    'vacuum': (0.0821, 0.14, 0.081),
    'magnetic_tool': (0.036, 0.14, 0.036),
    'sealant_tool': (0.05608, 0.14, 0.05608),
    'torque_tool': (0.049, 0.14, 0.049),
}

TOOL_APPEARANCE = {
    'vacuum': '0.16 0.47 0.72',
    'magnetic_tool': '0.43 0.30 0.65',
    'sealant_tool': '0.23 0.57 0.31',
    'torque_tool': '0.80 0.32 0.18',
}


def rotation_matrix(yaw_deg: float) -> tuple[tuple[float, float, float], ...]:
    """Rz(yaw) @ Rx(90): the OBJ Y axis becomes Webots Z."""
    angle = math.radians(yaw_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    return (
        (cosine, 0.0, sine),
        (sine, 0.0, -cosine),
        (0.0, 1.0, 0.0),
    )


def rotation_axis_angle(yaw_deg: float) -> tuple[float, float, float, float]:
    """Return the Webots axis-angle equivalent of :func:`rotation_matrix`."""
    matrix = rotation_matrix(yaw_deg)
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-12:
        return 0.0, 0.0, 1.0, 0.0
    if math.pi - angle < 1e-9:
        # 180 degrees: recover the axis from the symmetric part.
        diagonal = [matrix[i][i] for i in range(3)]
        axis = [math.sqrt(max(0.0, (diagonal[i] + 1.0) / 2.0)) for i in range(3)]
        return tuple(axis) + (angle,)
    scale = 2.0 * math.sin(angle)
    axis = (
        (matrix[2][1] - matrix[1][2]) / scale,
        (matrix[0][2] - matrix[2][0]) / scale,
        (matrix[1][0] - matrix[0][1]) / scale,
    )
    return tuple(axis) + (angle,)


def _vector(values) -> str:
    return ' '.join(format(value, '.12g') for value in values)


def _asset_path(asset_root: Path, name: str) -> Path:
    path = asset_root / ASSETS[name]
    if not path.is_file():
        raise FileNotFoundError(f'Webots asset does not exist: {path}')
    source = path.resolve()
    for line in source.read_text(encoding='utf-8', errors='replace').splitlines():
        parts = line.split()
        if parts and parts[0].lower() == 'mtllib':
            for material in parts[1:]:
                material_path = source.parent / material
                if not material_path.is_file():
                    raise FileNotFoundError(
                        f'Webots material does not exist: {material_path} '
                        f'(referenced by {source})'
                    )
    return path


def _mesh_shape(asset: Path, color: str, scale: float | None = None) -> str:
    shape = f'''Shape {{
  appearance PBRAppearance {{
    baseColor {color}
    roughness 0.5
    metalness 0.3
  }}
  geometry Mesh {{
    url [ {json.dumps(str(asset))} ]
  }}
}}'''
    if scale is None:
        return shape
    return f'''Transform {{
  scale {_vector((scale, scale, scale))}
  children [
{_indent(shape, 4)}
  ]
}}'''


def _indent(text: str, spaces: int) -> str:
    prefix = ' ' * spaces
    return '\n'.join(prefix + line for line in text.splitlines())


def _solid(
    *,
    name: str,
    translation,
    rotation,
    children: str,
    bounding: str,
    locked: bool,
    physics_mass: float | None = None,
) -> str:
    lines = [
        f'DEF {name} Solid {{',
        f'  translation {_vector(translation)}',
        f'  rotation {_vector(rotation)}',
        f'  name "{name}"',
    ]
    if locked:
        lines.append('  locked TRUE')
    lines.extend([
        '  children [',
        _indent(children, 4),
        '  ]',
        '  boundingObject ' + bounding,
    ])
    if physics_mass is not None:
        lines.append(
            f'  physics Physics {{ density -1 mass {format(physics_mass, ".12g")} }}'
        )
    lines.append('}')
    return '\n'.join(lines)


def _fixture_node(spec: ScenarioSpec, asset: Path) -> str:
    pose = spec.fixture
    bounding = '''Group {
  children [
    Pose {
      translation 0 -0.005 0
      children [ Box { size 0.35 0.01 0.22 } ]
    }
    Pose {
      translation -0.104 0.008 0
      children [ Box { size 0.014 0.016 0.156 } ]
    }
    Pose {
      translation 0.104 0.008 0
      children [ Box { size 0.014 0.016 0.156 } ]
    }
  ]
}'''
    return _solid(
        name='grant_fixture',
        translation=(pose.x, pose.y, pose.z),
        rotation=rotation_axis_angle(pose.yaw_deg),
        children=_mesh_shape(asset, '0.30 0.34 0.39'),
        bounding=bounding,
        locked=True,
    )


def _body_node(spec: ScenarioSpec, asset: Path) -> str:
    pose = spec.body
    bounding = '''Pose {
  translation 0 0.000695 0
  children [ Box { size 0.30 0.00915 0.17 } ]
}'''
    return _solid(
        name='grant_body',
        translation=(pose.x, pose.y, pose.z),
        rotation=rotation_axis_angle(pose.yaw_deg),
        children=_mesh_shape(asset, '0.64 0.68 0.72'),
        bounding=bounding,
        locked=True,
    )


def _rack_node(spec: ScenarioSpec, asset: Path) -> str:
    pose = spec.rack
    # The magazine model was validated with its local X turned around, so the
    # tool sockets face the robot. It still maps the OBJ Y axis to Webots Z.
    rotation = rotation_axis_angle(pose.yaw_deg + 180.0)
    bounding = '''Pose {
  translation 0 0.005 0
  children [ Box { size 0.42 0.01 0.14 } ]
}'''
    return _solid(
        name='grant_rack',
        translation=(pose.x, pose.y, pose.z),
        rotation=rotation,
        children=_mesh_shape(asset, '0.72 0.47 0.14'),
        bounding=bounding,
        locked=True,
    )


def _tool_node(tool, asset: Path) -> str:
    scale = 10.0 if tool.name == 'sealant_tool' else None
    bounding = f'''Pose {{
  translation 0 0.07 0
  children [ Box {{ size {_vector(TOOL_SIZE[tool.name])} }} ]
}}'''
    return _solid(
        name=TOOL_NODE_NAMES[tool.name],
        translation=(tool.pose.x, tool.pose.y, tool.pose.z),
        rotation=rotation_axis_angle(tool.pose.yaw_deg + 180.0),
        children=_mesh_shape(asset, TOOL_APPEARANCE[tool.name], scale=scale),
        bounding=bounding,
        locked=True,
    )


def _tray_node(spec: ScenarioSpec, asset: Path) -> str:
    pose = spec.tray_pose
    assert pose is not None
    bounding = '''Group {
  children [
    Pose {
      translation 0 0.0012 0
      children [ Box { size 0.12 0.0024 0.07 } ]
    }
    Pose {
      translation -0.059 0.005 0
      children [ Box { size 0.002 0.01 0.07 } ]
    }
    Pose {
      translation 0.059 0.005 0
      children [ Box { size 0.002 0.01 0.07 } ]
    }
    Pose {
      translation 0 0.005 -0.034
      children [ Box { size 0.12 0.01 0.002 } ]
    }
    Pose {
      translation 0 0.005 0.034
      children [ Box { size 0.12 0.01 0.002 } ]
    }
  ]
}'''
    return _solid(
        name='grant_tray',
        translation=(pose.x, pose.y, pose.z),
        rotation=rotation_axis_angle(pose.yaw_deg),
        children=_mesh_shape(asset, '0.25 0.25 0.28'),
        bounding=bounding,
        locked=True,
    )


def _screw_node(index: int, screw, asset: Path) -> str:
    match = re.fullmatch(r'screw_(\d+)', screw.name)
    number = int(match.group(1)) if match else index
    bounding = '''Pose {
  translation 0 0.0045 0
  children [ Box { size 0.0055 0.009 0.0055 } ]
}'''
    return _solid(
        name=f'grant_screw_{number:02d}',
        translation=(screw.pose.x, screw.pose.y, screw.pose.z),
        rotation=rotation_axis_angle(screw.pose.yaw_deg),
        children=_mesh_shape(asset, '0.62 0.64 0.67'),
        bounding=bounding,
        locked=True,
    )


def _cap_node(spec: ScenarioSpec, asset: Path) -> str:
    pose = spec.cap_pose
    assert pose is not None
    bounding = '''Pose {
  translation 0 0.004165 0
  children [ Box { size 0.1384 0.00367 0.0984 } ]
}'''
    installed = spec.cap_state == 'installed'
    return _solid(
        name='grant_cap',
        translation=(pose.x, pose.y, pose.z),
        rotation=rotation_axis_angle(pose.yaw_deg),
        children=_mesh_shape(asset, '0.78 0.82 0.86'),
        bounding=bounding,
        locked=installed,
        physics_mass=None if installed else 0.1,
    )


def build_nodes(spec: ScenarioSpec, asset_root: Path | str) -> list[str]:
    """Return the deterministic Webots node strings for one experiment."""
    root = Path(asset_root)
    if not root.is_dir():
        raise FileNotFoundError(f'Webots asset directory does not exist: {root}')

    required = ['fixture', 'body', 'rack']
    required.extend(tool.name for tool in spec.tools)
    if spec.tray_pose is not None:
        required.extend(['tray', 'screw'])
    if spec.cap_state != 'absent':
        required.append('cap')
    assets = {name: _asset_path(root, name) for name in required}

    nodes = [
        _fixture_node(spec, assets['fixture']),
        _body_node(spec, assets['body']),
        _rack_node(spec, assets['rack']),
    ]
    nodes.extend(_tool_node(tool, assets[tool.name]) for tool in spec.tools)
    if spec.tray_pose is not None:
        nodes.append(_tray_node(spec, assets['tray']))
        nodes.extend(
            _screw_node(index, screw, assets['screw'])
            for index, screw in enumerate(spec.screws, start=1)
        )
    if spec.cap_state != 'absent':
        nodes.append(_cap_node(spec, assets['cap']))
    return nodes
