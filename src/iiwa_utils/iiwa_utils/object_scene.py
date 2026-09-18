"""Validated object configuration and Webots scene generation (metres/radians)."""
import json
import math
import random
import shlex
from dataclasses import dataclass, field
from pathlib import Path

from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class ObjectModelCfg:
    path: str
    rotation_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ObjectsCfg:
    enabled: bool = False
    count: int = 0
    models: list[ObjectModelCfg] = field(default_factory=list)
    area: dict[str, list[float]] = field(default_factory=dict)


def parse_objects(raw, resolve_path) -> ObjectsCfg:
    if raw is None:
        return ObjectsCfg()
    if not isinstance(raw, dict):
        raise ValueError('digital_twin.webots.objects must be a mapping')
    enabled = raw.get('enabled', False)
    count = raw.get('count', 0)
    models = raw.get('models', [])
    area = raw.get('area', {})
    if type(enabled) is not bool:
        raise ValueError('objects.enabled must be true or false')
    if type(count) is not int or count < 0:
        raise ValueError('objects.count must be an integer >= 0')
    if not isinstance(models, list):
        raise ValueError('objects.models must be a list of paths or model mappings')
    if enabled and count and not models:
        raise ValueError('objects.models must not be empty when count > 0')
    if not isinstance(area, dict):
        raise ValueError('objects.area must be a mapping')
    parsed_area = {}
    if area or (enabled and count):
        for axis in 'xyz':
            bounds = area.get(axis)
            if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in bounds)
                    or bounds[0] > bounds[1]):
                raise ValueError(f'objects.area.{axis} must be [min, max], finite metres, min <= max')
            parsed_area[axis] = list(map(float, bounds))
    resolved = []
    for index, entry in enumerate(models):
        if isinstance(entry, str):
            entry = {'path': entry}
        if not isinstance(entry, dict) or entry.keys() - {'path', 'rotation_deg'}:
            raise ValueError(f'objects.models[{index}] requires path and optional rotation_deg')
        path = entry.get('path')
        angles = entry.get('rotation_deg', (0, 0, 0))
        if not isinstance(path, str) or not path.strip():
            raise ValueError(f'objects.models[{index}].path must be a nonempty path')
        if (not isinstance(angles, (list, tuple)) or len(angles) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in angles)):
            raise ValueError(f'objects.models[{index}].rotation_deg must be finite [X, Y, Z] degrees')
        resolved.append(ObjectModelCfg(resolve_path(path), tuple(map(float, angles))))
    if any(Path(model.path).suffix.lower() != '.obj' for model in resolved):
        raise ValueError('objects.models currently supports Wavefront .obj files')
    return ObjectsCfg(enabled, count, resolved, parsed_area)


def robot_pose(translation, rotation):
    try:
        position = tuple(map(float, translation.split()))
        orient = tuple(map(float, rotation.split()))
    except (ValueError, AttributeError) as exc:
        raise ValueError('Robot transform/rotation must be numeric strings') from exc
    if len(position) != 3 or len(orient) != 4 or not all(map(math.isfinite, position + orient)):
        raise ValueError('Robot transform needs 3 and rotation 4 finite numbers')
    length = math.sqrt(sum(v*v for v in orient[:3]))
    if length == 0:
        raise ValueError('Robot rotation axis must not be zero')
    return position, tuple(v / length for v in orient[:3]) + (orient[3],)


def world_position(local, translation, rotation):
    position, orient = robot_pose(translation, rotation)
    axis, angle = orient[:3], orient[3]
    c, s = math.cos(angle), math.sin(angle)
    dot = sum(a*b for a, b in zip(axis, local))
    cross = (axis[1]*local[2] - axis[2]*local[1],
             axis[2]*local[0] - axis[0]*local[2],
             axis[0]*local[1] - axis[1]*local[0])
    return tuple(position[i] + local[i]*c + cross[i]*s + axis[i]*dot*(1-c) for i in range(3))


@dataclass(frozen=True)
class ObjAsset:
    path: Path
    offset: tuple
    size: tuple
    textures: tuple


def _require_file(path):
    if not path.is_file():
        raise ValueError(f'Object asset file does not exist: {path}')


def load_obj(path) -> ObjAsset:
    """Keep original OBJ/MTL/UV data; derive a centred, bottom-aligned origin."""
    path = Path(path).resolve()
    _require_file(path)
    low, high = [math.inf]*3, [-math.inf]*3
    materials, textures = [], []
    for line in path.read_text(encoding='utf-8').splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'v':
            vertex = tuple(map(float, parts[1:4]))
            if len(vertex) != 3 or not all(map(math.isfinite, vertex)):
                raise ValueError(f'Invalid vertex in {path}')
            low = [min(a, b) for a, b in zip(low, vertex)]
            high = [max(a, b) for a, b in zip(high, vertex)]
        elif parts[0] == 'mtllib':
            materials.extend(path.parent / p for p in shlex.split(line)[1:])
    size = tuple(b-a for a, b in zip(low, high))
    if any(not math.isfinite(v) or v <= 0 for v in size):
        raise ValueError(f'OBJ must have finite, nonzero 3D bounds: {path}')
    for material in materials:
        _require_file(material)
        for line in material.read_text(encoding='utf-8').splitlines():
            parts = shlex.split(line, comments=True)
            if parts and (parts[0].lower().startswith('map_') or parts[0].lower() in ('bump', 'disp', 'decal', 'norm')):
                # Wavefront map options precede the final texture filename.
                texture = material.parent / parts[-1]
                _require_file(texture)
                textures.append(texture)
    offset = (-(low[0]+high[0])/2, -(low[1]+high[1])/2, -low[2])
    return ObjAsset(path, offset, size, tuple(textures))


def _vector(values):
    return ' '.join(format(v, '.12g') for v in values)


def model_rotation(rotation_deg):
    """Fixed-axis XYZ degrees: Rz(z) @ Ry(y) @ Rx(x), relative to robot base."""
    rotvec = Rotation.from_euler('xyz', rotation_deg, degrees=True).as_rotvec()
    angle = math.sqrt(sum(v*v for v in rotvec))
    if angle < 1e-12:
        return (0.0, 0.0, 1.0, 0.0)
    return tuple(float(v / angle) for v in rotvec) + (angle,)


def build_objects(config, translation, rotation, rng=None):
    if not config.enabled or config.count == 0:
        return []
    _, orient = robot_pose(translation, rotation)
    # Validate every allowed asset before any service request is sent.
    assets = [(load_obj(model.path), model.rotation_deg) for model in config.models]
    rng = rng or random.Random()
    nodes = []
    for index in range(config.count):
        asset, angles = rng.choice(assets)
        model_orient = model_rotation(angles)
        matrix = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
        # Lift the rotated collision box above the spawn anchor. Keep the
        # original oriented box, rather than inflating it into a world AABB.
        half_height = sum(abs(matrix[2, i]) * asset.size[i] / 2 for i in range(3))
        center = (-asset.offset[0], -asset.offset[1],
                  -asset.offset[2] + asset.size[2] / 2)
        rotated_center = matrix @ center
        visual_offset = tuple(-rotated_center[i] + (half_height if i == 2 else 0)
                              for i in range(3))
        local = tuple(rng.uniform(*config.area[axis]) for axis in 'xyz')
        world = world_position(local, translation, rotation)
        nodes.append(f'''Solid {{
  name "spawned_object_{index:04d}"
  translation {_vector(world)}
  rotation {_vector(orient)}
  children [ Pose {{
    translation {_vector(visual_offset)}
    rotation {_vector(model_orient)}
    children [ CadShape {{ url [ {json.dumps(str(asset.path))} ] }} ]
  }} ]
  boundingObject Pose {{
    translation 0 0 {half_height:.12g}
    rotation {_vector(model_orient)}
    children [ Box {{ size {_vector(asset.size)} }} ]
  }}
  physics Physics {{ density -1 mass 0.5 }}
}}''')
    return nodes
