"""Configuration, model placement and textured asset regressions."""
import math
import random
from pathlib import Path

import pytest

from iiwa_utils import object_scene as scene


@pytest.fixture
def model(tmp_path):
    (tmp_path / 'texture.png').write_bytes(b'texture')
    (tmp_path / 'hammer.mtl').write_text('newmtl wood\nmap_Kd texture.png\n')
    path = tmp_path / 'hammer.obj'
    path.write_text('mtllib hammer.mtl\nv 10 20 30\nv 12 24 36\nf 1 2 1\n')
    return path


def config(model, **updates):
    raw = dict(enabled=True, count=5, models=[str(model)],
               area=dict(x=[1, 1], y=[2, 2], z=[3, 3]))
    raw.update(updates)
    return scene.parse_objects(raw, lambda p: p)


def test_default_disabled():
    assert not scene.parse_objects(None, lambda p: p).enabled


@pytest.mark.parametrize('updates', [
    {'count': -1}, {'count': True}, {'count': 1.5}, {'enabled': 'true'},
    {'models': []}, {'models': 'hammer.obj'},
    {'area': {'x': [2, 1], 'y': [0, 1], 'z': [0, 1]}},
    {'area': {'x': [0, math.nan], 'y': [0, 1], 'z': [0, 1]}},
    {'area': {'x': [False, 1], 'y': [0, 1], 'z': [0, 1]}},
])
def test_reject_bad_configuration(model, updates):
    with pytest.raises(ValueError):
        config(model, **updates)


def test_rotation_about_translated_robot():
    assert scene.world_position((1, 2, 3), '10 20 30', f'0 0 2 {math.pi/2}') == pytest.approx((8, 21, 33))
    assert scene.world_position((0, 1, 0), '0 0 0', f'1 0 0 {math.pi/2}') == pytest.approx((0, 0, 1))


@pytest.mark.parametrize('translation,rotation', [('0 0', '0 0 1 0'), ('0 nan 0', '0 0 1 0'), ('0 0 0', '0 0 0 1')])
def test_bad_robot_pose(translation, rotation):
    with pytest.raises(ValueError):
        scene.world_position((0, 0, 0), translation, rotation)


def test_exact_count_repeats_and_centered_geometry(model):
    nodes = scene.build_objects(config(model), '10 20 30', '0 0 1 0', random.Random(1))
    assert len(nodes) == 5
    assert len(set(nodes)) == 5
    for node in nodes:
        assert 'translation 11 22 33' in node
        assert 'CadShape' in node and str(model) in node
        assert 'translation -11 -22 -30' in node  # XY centered, bottom at local z=0
        assert 'size 2 4 6' in node
    assert scene.build_objects(config(model, count=0), '0 0 0', '0 0 1 0') == []


def test_missing_texture_fails_before_spawning(model):
    (model.parent / 'texture.png').unlink()
    with pytest.raises(ValueError, match='texture'):
        scene.build_objects(config(model), '0 0 0', '0 0 1 0')


def test_repository_models_have_materials_and_textures():
    root = Path(__file__).resolve().parents[2] / 'iiwa_description/objects/hammer'
    models = sorted(root.glob('*.obj'))
    assert len(models) == 5
    for path in models:
        asset = scene.load_obj(path)
        assert all(size > 0 for size in asset.size)
        assert asset.textures


def test_per_model_rotation_and_serialization(model):
    from dataclasses import asdict
    cfg = config(model, models=[str(model), {'path': str(model), 'rotation_deg': [90, 0, 180]}])
    assert cfg.models[0].rotation_deg == (0.0, 0.0, 0.0)
    assert cfg.models[1].rotation_deg == (90.0, 0.0, 180.0)
    assert scene.parse_objects(asdict(cfg), lambda p: p) == cfg


@pytest.mark.parametrize('entry', [
    {}, {'path': 12}, {'path': 'a.obj', 'rotation_deg': [0, 0]},
    {'path': 'a.obj', 'rotation_deg': [True, 0, 0]},
    {'path': 'a.obj', 'rotation_deg': [0, float('inf'), 0]},
    {'path': 'a.obj', 'rotation_deg': '90 0 0'},
    {'path': 'a.obj', 'rotaton_deg': [90, 0, 0]},
])
def test_invalid_model_orientation(model, entry):
    with pytest.raises(ValueError):
        config(model, models=[entry])


def test_quarter_turn_repositions_geometry_and_collision(model):
    cfg = config(model, count=1, models=[{'path': str(model), 'rotation_deg': [90, 0, 0]}])
    node = scene.build_objects(cfg, '10 20 30', f'0 0 1 {math.pi/2}')[0]
    # Anchor (1,2,3) rotates about the ROBOT, not about the model's X axis.
    assert 'translation 8 21 33' in node
    # Raw centre (11,22,33) -> (11,-33,22); rotated half-height = 2.
    assert 'translation -11 33 -20' in node
    assert 'translation 0 0 2' in node
    # Rendering AND the original 2x4x6 collision box receive the model rotation.
    assert node.count('rotation 1 0 0 1.57079632679') == 2
    assert 'size 2 4 6' in node


def test_rotation_order_xyz():
    # Apply Rx(90), then Ry(0), then Rz(90): x -> y, y -> z, z -> x.
    orient = scene.model_rotation([90, 0, 90])
    encoded = ' '.join(map(str, orient))
    assert scene.world_position((1, 0, 0), '0 0 0', encoded) == pytest.approx((0, 1, 0))
    assert scene.world_position((0, 1, 0), '0 0 0', encoded) == pytest.approx((0, 0, 1))
    assert scene.world_position((0, 0, 1), '0 0 0', encoded) == pytest.approx((1, 0, 0))
