"""The world template contains the shared table and one attachment supervisor."""
import re
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
WORLD = PACKAGE / 'worlds' / 'grant' / 'iiwa-grant.wbt'


def test_world_matches_base_layout():
    text = WORLD.read_text(encoding='utf-8')
    assert '#VRML_SIM R2025a utf8' in text
    # Webots defaults to 32 ms when the field is omitted; both forms keep the
    # deterministic simulation step.
    assert 'basicTimeStep 32' in text or 'basicTimeStep' not in text
    assert re.search(r'CollaborativeRobotTable \{\s+translation 0 0 -0\.007', text)
    assert text.count('WorkspaceLimiter {') == 2
    assert 'WorkspaceLimiter_back' in text
    assert 'WorkspaceLimiter_right' in text
    assert 'coulombFriction' in text and '0.6' in text
    assert 'Object.proto' not in text


def test_world_has_only_the_attachment_supervisor_robot():
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
    assert 'GrantToolRack' not in text
    robots = re.findall(r'Robot \{', text)
    assert len(robots) == 1
    assert 'name "GrantAttachmentSupervisor"' in text
    assert 'controller "<extern>"' in text
    assert 'supervisor TRUE' in text
    assert 'name "Ros2Supervisor"' not in text


def test_world_only_references_the_two_local_protos():
    text = WORLD.read_text(encoding='utf-8')
    externprotos = re.findall(r'EXTERNPROTO "([^"]+)"', text)
    local = [url for url in externprotos if not url.startswith(('http://', 'https://'))]
    assert len(local) == 2
    assert local[0].endswith('CollaborativeRobotTable.proto')
    assert local[1].endswith('WorkspaceLimiter.proto')
    for url in externprotos:
        if url.startswith(('http://', 'https://')):
            assert '/webots/R2025a/' in url
