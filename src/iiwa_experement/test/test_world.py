"""The world template contains only the shared table, fences and lighting."""
import re
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
WORLD = PACKAGE / 'worlds' / 'grant' / 'iiwa-grant.wbt'


def test_world_matches_base_layout():
    text = WORLD.read_text(encoding='utf-8')
    assert '#VRML_SIM R2025a utf8' in text
    assert 'basicTimeStep 32' in text
    assert re.search(r'CollaborativeRobotTable \{\s+translation 0 0 -0\.007', text)
    assert text.count('WorkspaceLimiter {') == 2
    assert 'WorkspaceLimiter_back' in text
    assert 'WorkspaceLimiter_right' in text
    assert 'coulombFriction [ 0.6 ]' in text
    assert 'Object.proto' not in text


def test_world_has_no_preplaced_experiment_objects_or_robot():
    text = WORLD.read_text(encoding='utf-8')
    for asset in (
        'stapel.obj',
        'base-detail.obj',
        'cap.obj',
        'lotok.obj',
        'vint.obj',
        'tool-magizine.obj',
        'vacum-tool.obj',
        'magnit-tool.obj',
        'germetic-tool.obj',
        'wranch-tool.obj',
    ):
        assert asset not in text
    assert 'Kuka' not in text
    assert 'Robot {' not in text
    assert 'GrantToolRack' not in text


def test_world_only_references_two_local_protos():
    text = WORLD.read_text(encoding='utf-8')
    externprotos = re.findall(r'EXTERNPROTO "([^"]+)"', text)
    local = [url for url in externprotos if not url.startswith(('http://', 'https://'))]
    assert local == [
        '../../../iiwa_description/resource/protos/CollaborativeRobotTable.proto',
        '../../../iiwa_description/resource/protos/WorkspaceLimiter.proto',
    ]
    for url in externprotos:
        if url.startswith(('http://', 'https://')):
            assert '/webots/R2025a/' in url
