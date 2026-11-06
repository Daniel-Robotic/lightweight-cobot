"""Both grant hatch worlds include the four-slot magazine and its tools."""
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2] / 'iiwa_description'
WORLDS = ROOT / 'worlds/grant'
RACK = ROOT / 'resource/protos/GrantToolRack.proto'
BRINGUP = ROOT.parent / 'iiwa_bringup/launch'


def pose(world: str, name: str):
    match = re.search(rf'DEF {name} Solid \{{\s+translation ([^\n]+)', world)
    assert match, name
    return tuple(map(float, match.group(1).split()))


def test_both_worlds_have_magazine_next_to_work_zone():
    for world_name in ('body_hatch.wbt', 'body_hatch_installed.wbt'):
        world_path = WORLDS / world_name
        world = world_path.read_text()
        assert 'GrantToolRack {' in world
        assert (world_path.parent / '../../resource/protos/GrantToolRack.proto').is_file()
        rack = re.search(r'GrantToolRack \{\s+translation ([^\n]+)', world)
        assert rack
        x, y, z = map(float, rack.group(1).split())
        assert x > -0.25 and abs(y) < 0.54 and abs(z - 0.793) < 1e-6
        assert 'rotation 0 0.70710678118 0.70710678118 3.14159265359' in world


def test_magazine_contains_all_four_grant_tools():
    text = RACK.read_text()
    for asset in ('tool-magizine.obj', 'vacum-tool.obj', 'magnit-tool.obj',
                  'germetic-tool.obj', 'wranch-tool.obj'):
        assert text.count(asset) == 1
        assert (RACK.parent / '../../objects/grant' / asset).is_file()
    for station in ('-0.135', '-0.045', '0.045', '0.135'):
        assert f'translation {station} 0.01 0' in text
    assert 'scale 4 4 4' not in text
    assert text.count('scale 10 10 10') == 1
    assert 'Box { size 0.42 0.01 0.14 }' in text
    vacuum = (RACK.parent / '../../objects/grant/vacum-tool.obj').read_text()
    assert 'Hollow_elastomer_vacuum_cup_OD_81.0_mm' in vacuum
    assert 'Black_sealant_cartridge_body' not in vacuum


def test_installed_world_puts_hatch_on_body():
    world = (WORLDS / 'body_hatch_installed.wbt').read_text()
    body = pose(world, 'BODY_HATCH_BODY')
    cap = pose(world, 'BODY_HATCH_CAP')
    assert cap[:2] == body[:2]
    assert abs(cap[2] + 0.002333 - (body[2] + 0.005273)) < 1e-6
    cap_node = world.split('DEF BODY_HATCH_CAP Solid {', 1)[1].split('\n}', 1)[0]
    assert 'locked TRUE' in cap_node
    assert 'physics Physics {' not in cap_node


def test_robot_spawns_without_a_preinstalled_vacuum_tool():
    main_launch = (BRINGUP / 'iiwa.launch.py').read_text()
    controllers = (BRINGUP / 'supported/controllers.launch.py').read_text()
    assert 'body_hatch_vacuum' not in main_launch
    assert 'body_hatch_vacuum' not in controllers
    assert 'body_hatch_robot_spawner' not in controllers
    assert 'spawner_urdf = URDFSpawner(' in controllers


def test_loose_hatch_remains_separate_and_movable():
    world = (WORLDS / 'body_hatch.wbt').read_text()
    cap = world.split('DEF BODY_HATCH_CAP Solid {', 1)[1].split('\n}', 1)[0]
    body = world.split('DEF BODY_HATCH_BODY Solid {', 1)[1].split('\n}', 1)[0]
    cap_pose = re.search(r'translation ([^\n]+)', cap).group(1).split()
    body_pose = re.search(r'translation ([^\n]+)', body).group(1).split()
    assert abs(float(cap_pose[1]) - float(body_pose[1])) > 0.2
    assert 'physics Physics {' in cap
    assert 'locked TRUE' not in cap
