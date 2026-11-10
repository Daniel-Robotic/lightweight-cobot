"""The MoveIt scene must mirror Webots and keep attachments exclusive."""
from pathlib import Path

import numpy as np
import pytest
from moveit_msgs.msg import CollisionObject, PlanningScene

from iiwa_experement.geometry import Box, Transform, load_geometry, world_target
from iiwa_experement.planning_scene import (
    PlanningSceneBridge,
    SceneObject,
    attached_cap,
    build_collision_matrix,
    load_srdf_disabled_pairs,
    attached_screw,
    attached_tool,
    build_world_objects,
    collision_object,
    placed_cap_pose,
    scene_ids,
    update_contact_entry,
)
from iiwa_experement.scenario import load_scenario


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / 'config' / 'scenes.yaml'
GEOMETRY = PACKAGE / 'config' / 'geometry.yaml'


def spec(experiment_id):
    return load_scenario(experiment_id, SCENES)


class FakeBridge(PlanningSceneBridge):
    def __init__(self, experiment_id=11):
        geometry = load_geometry(GEOMETRY)
        scenario = spec(experiment_id)
        self.geometry = geometry
        self.spec = scenario
        self.base_to_world = geometry.world_to_base.inverse()
        self._world_objects = {
            obj.object_id: obj for obj in build_world_objects(scenario, geometry)
        }
        self._initial_world = dict(self._world_objects)
        self._attached = {}
        self._disabled_pairs = []
        self._acm_names = []
        self._acm_values = []
        self._latest_joint_state = None
        self.node = None
        self.last_scene = PlanningScene()
        self.published = []

    def _publish(self, scene):
        self.published.append(scene)

    def assert_synchronized(self, *args, **kwargs):
        return None


def test_initial_scene_contains_every_experiment_object():
    objects = build_world_objects(spec(11), load_geometry(GEOMETRY))
    ids = {obj.object_id for obj in objects}
    assert {'grant_table', 'grant_fixture', 'grant_fixture_rail_a',
            'grant_fixture_rail_b', 'grant_body', 'grant_rack',
            'grant_tray', 'grant_cap', 'grant_magnetic_tool'} <= ids
    assert {f'grant_screw_{index:02d}' for index in range(1, 11)} <= ids
    assert 'grant_vacuum' not in ids
    # table, fixture base, two fixture rails, body, rack, tray base, four
    # tray walls, ten screws, installed hatch and one magnetic tool.
    assert len(ids) == 23


def test_experiment_nineteen_moves_tray_and_screws():
    standard = {obj.object_id: obj for obj in build_world_objects(
        spec(11), load_geometry(GEOMETRY))}
    shifted = {obj.object_id: obj for obj in build_world_objects(
        spec(19), load_geometry(GEOMETRY))}
    for index in range(1, 11):
        name = f'grant_screw_{index:02d}'
        assert shifted[name].pose.position == pytest.approx(
            standard[name].pose.position + np.array([-0.02, 0.02, 0.0]),
            abs=1e-9,
        )


def test_collision_boxes_match_measured_meshes():
    objects = {obj.object_id: obj for obj in build_world_objects(
        spec(16), load_geometry(GEOMETRY))}
    assert objects['grant_body'].box.size == pytest.approx((0.30, 0.17, 0.00915))
    assert objects['grant_cap'].box.size == pytest.approx((0.1384, 0.0984, 0.00267))
    assert objects['grant_screw_01'].box.size == pytest.approx((0.0055, 0.0055, 0.009))
    assert objects['grant_vacuum'].box.size[2] == pytest.approx(0.140)


def test_collision_object_is_in_the_robot_model_frame():
    pose = Transform.from_xyz_rpy([0.0, 0.0, 0.0], [0, 0, 0], parent='world', child='x')
    message = collision_object('obj', Box((0.1, 0.2, 0.3)), pose)
    assert message.header.frame_id == 'base_link'
    assert message.id == 'obj'
    assert list(message.primitives[0].dimensions) == pytest.approx([0.1, 0.2, 0.3])


def test_attach_removes_the_world_object_and_adds_one_attachment():
    bridge = FakeBridge(16)
    tool = attached_tool('vacuum', bridge.geometry)
    bridge.attach(tool)
    assert tool.object_id not in bridge._world_objects
    assert bridge._attached[tool.object_id] is tool
    scene = bridge.published[-1]
    assert scene.is_diff is False
    assert tool.object_id not in [obj.id for obj in scene.world.collision_objects]
    assert len(scene.robot_state.attached_collision_objects) == 1
    assert scene.robot_state.attached_collision_objects[0].object.id == tool.object_id
    assert scene.robot_state.attached_collision_objects[0].link_name == 'patron'
    assert 'tcp' in scene.robot_state.attached_collision_objects[0].touch_links


def test_detach_reverses_the_attachment():
    bridge = FakeBridge(16)
    tool = attached_tool('vacuum', bridge.geometry)
    bridge.attach(tool)
    bridge.detach(tool.object_id, bridge.geometry.world_to_base.compose_relaxed(
        Transform.from_xyz_rpy([0, 0, 0], [0, 0, 0], parent='world', child='x')
    ))
    assert tool.object_id in bridge._world_objects
    assert tool.object_id not in bridge._attached
    scene = bridge.published[-1]
    assert not scene.robot_state.attached_collision_objects
    assert tool.object_id in [obj.id for obj in scene.world.collision_objects]


def test_handoff_removes_only_the_delivered_screw():
    bridge = FakeBridge(11)
    screw = attached_screw('screw_1', bridge.geometry)
    bridge.attach(screw)
    assert 'grant_screw_02' in bridge._world_objects
    bridge.remove(screw.object_id)
    assert screw.object_id not in bridge._attached
    assert screw.object_id not in bridge._world_objects
    assert 'grant_screw_02' in bridge._world_objects


def test_cap_and_screw_attachment_geometry_uses_tcp_offsets():
    geometry = load_geometry(GEOMETRY)
    cap = attached_cap(geometry)
    screw = attached_screw('screw_3', geometry)
    # The boxes stop 1 mm short of the support surface at the grasp pose.
    assert cap.relative_pose.xyz == pytest.approx((0.0, 0.0, 0.141335 + 0.0552))
    assert cap.box.size == pytest.approx((0.1384, 0.0984, 0.00267))
    assert screw.relative_pose.xyz == pytest.approx((0.0, 0.0, 0.144 + 0.0552))
    assert screw.box.size == pytest.approx((0.0055, 0.0055, 0.008))


def test_contact_permission_changes_exactly_one_pair():
    names = ['link1', 'link2', 'tool']
    values = [[True, False, False], [False, True, False], [False, False, True]]
    new_names, matrix = update_contact_entry(names, values, 'tool', 'link2', True)
    assert new_names == names
    assert matrix[2][1] is True and matrix[1][2] is True
    assert matrix[2][0] is False and matrix[0][2] is False
    assert matrix[0][1] is False


def test_restore_world_returns_the_calibrated_box_pose():
    bridge = FakeBridge(16)
    tool = attached_tool('vacuum', bridge.geometry)
    bridge.attach(tool)
    bridge.restore_world(tool.object_id)
    assert tool.object_id not in bridge._attached
    restored = bridge._world_objects[tool.object_id]
    expected = {
        obj.object_id: obj
        for obj in build_world_objects(bridge.spec, bridge.geometry)
    }[tool.object_id]
    assert restored.pose.xyz == pytest.approx(expected.pose.xyz)
    assert restored.pose.rotation == expected.pose.rotation
    with pytest.raises(Exception):
        bridge.restore_world('grant_unknown')


def test_placed_cap_box_is_flat_and_matches_the_preinstalled_cap():
    geometry = load_geometry(GEOMETRY)
    scenario = spec(1)
    seat = world_target(scenario, geometry, 'cap_seat')
    placed = placed_cap_pose(seat, scenario.body.yaw_deg)
    # The box is flat (yaw only) and centred on the plate, like the
    # pre-installed hatch of experiments 11-15.
    preinstalled = {
        obj.object_id: obj
        for obj in build_world_objects(spec(11), geometry)
    }['grant_cap']
    assert placed.xyz == pytest.approx(preinstalled.pose.xyz)
    assert np.asarray(placed.rotation) == pytest.approx(
        np.asarray(preinstalled.pose.rotation)
    )


def test_srdf_collision_matrix_allows_only_disabled_pairs():
    root = Path(__file__).resolve().parents[2]
    srdf = root / 'iiwa_config' / 'config' / 'moveit' / 'iiwa7.srdf'
    pairs = load_srdf_disabled_pairs(srdf)
    assert ('link1', 'link2') in pairs
    names, matrix = build_collision_matrix(
        ['grant_body', 'grant_tool'], disabled_pairs=pairs,
    )
    assert 'base_link' in names and 'grant_body' in names
    def allowed(first, second):
        return matrix[names.index(first)][names.index(second)]
    assert allowed('link1', 'link2') is True
    assert allowed('base_link', 'link1') is True
    assert allowed('link1', 'grant_body') is False
    assert allowed('grant_body', 'grant_tool') is False
    assert allowed('grant_body', 'grant_body') is True


def test_scene_ids_reports_world_and_attached_sets():
    scene = PlanningScene()
    scene.world.collision_objects.append(CollisionObject(id='world_a'))
    from moveit_msgs.msg import AttachedCollisionObject

    attached = AttachedCollisionObject()
    attached.object.id = 'attached_a'
    scene.robot_state.attached_collision_objects.append(attached)
    world, held = scene_ids(scene)
    assert world == {'world_a'}
    assert held == {'attached_a'}
