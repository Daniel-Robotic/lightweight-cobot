"""Skill registry: the only place where a graph node becomes robot motion."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from iiwa_experement.geometry import (
    GeometrySpec,
    Transform,
    sealant_point,
    tcp_goal,
    world_target,
)
from iiwa_experement.scenario import ScenarioSpec
from iiwa_scene_context.catalog import DEFAULT_CATALOG, SkillCatalog

from .schema import PlanNode
from .skill_context import SkillContext


class SkillError(RuntimeError):
    def __init__(self, event: str, message: str) -> None:
        super().__init__(message)
        self.event = event


@dataclass(frozen=True)
class SkillOutcome:
    event: str
    details: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {'event': self.event, 'details': dict(self.details)}


def _translation(xyz) -> Transform:
    return Transform.from_xyz_rpy(
        [float(value) for value in xyz], [0.0, 0.0, 0.0],
        parent='tcp', child='offset',
    )


def _above(goal: Transform, gap: float) -> Transform:
    return goal.compose_relaxed(_translation([0.0, 0.0, -gap]))


def _distance(first: Transform, second: Transform) -> float:
    return float(np.linalg.norm(first.position - second.position))


def _angle(first: Transform, second: Transform) -> float:
    cosine = (np.trace(first.matrix.T @ second.matrix) - 1.0) / 2.0
    return float(np.arccos(max(-1.0, min(1.0, cosine))))


def _state_pose(entry: Any, child: str) -> Transform | None:
    if not isinstance(entry, dict):
        return None
    xyz = entry.get('xyz')
    rotation = entry.get('rotation')
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        return None
    if not isinstance(rotation, (list, tuple)) or len(rotation) != 3:
        return None
    try:
        return Transform(
            xyz=tuple(float(value) for value in xyz),
            rotation=tuple(tuple(float(value) for value in row) for row in rotation),
            parent='world',
            child=child,
        )
    except (TypeError, ValueError):
        return None


def object_definition_name(object_id: str) -> str | None:
    if object_id == 'cap':
        return 'grant_cap'
    if object_id.startswith('screw_') and object_id.split('_')[-1].isdigit():
        return f'grant_screw_{int(object_id.split("_")[-1]):02d}'
    for name in ('vacuum', 'magnetic_tool', 'sealant_tool', 'torque_tool'):
        if object_id == name:
            return f'grant_{name}'
    return None


class SkillRegistry:
    def __init__(
        self,
        motion: Any,
        supervisor: Any,
        *,
        catalog: SkillCatalog = DEFAULT_CATALOG,
        scene_adapter: Any = None,
    ) -> None:
        self.motion = motion
        self.supervisor = supervisor
        self.catalog = catalog
        self.scene_adapter = scene_adapter
        self._handlers = {
            'detect': self._detect,
            'approach': self._approach_skill,
            'lower': self._lower_skill,
            'grab': self._grab,
            'pick': self._grab,
            'lift': self._lift,
            'transfer': self._transfer,
            'move_to_storage': self._move_to_storage,
            'align': self._align,
            'release': self._release,
            'retreat': self._retreat,
            'capture': self._capture,
            'verify': self._verify,
            'inspect': self._verify,
            'stop': self._stop,
            'wait_scene_clear': self._wait_scene_clear,
            'inspect_state': self._inspect_state,
            'notify_operator': self._notify_operator,
            'dispense': self._dispense,
            'dock_tool': self._dock_tool,
            'handoff_screw': self._handoff_screw,
        }

    # ------------------------------------------------------------------ dispatch
    def execute(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        handler = self._handlers.get(node.skill)
        if handler is None:
            return SkillOutcome('failed', {'reason': f'unknown skill {node.skill!r}'})
        try:
            return handler(node, context)
        except SkillError as error:
            return SkillOutcome(error.event, {'reason': str(error)})
        except Exception as error:  # never turn an unexpected exception into success
            return SkillOutcome('failed', {'reason': str(error)})

    # ------------------------------------------------------------------ motion
    def _state(self) -> dict[str, Any]:
        try:
            state = self.supervisor.latest()
        except Exception:
            return {}
        return state if isinstance(state, dict) else {}

    def _refresh_live_ownership(self, context: SkillContext) -> str | None:
        """Read the tool and held object from the live supervisor.

        Context memory is not authoritative between commands or runs: the
        supervisor may have been changed by another accepted graph or by a
        previous failed attempt.
        """
        state = self._state()
        if not state:
            return 'supervisor state is unavailable'
        context.supervisor_state = state
        mounted = state.get('mounted_tool')
        held = state.get('held_object')
        context.mounted_tool = mounted if isinstance(mounted, str) else None
        context.held_object = held if isinstance(held, str) else None
        return None

    def _move(
        self,
        context: SkillContext,
        goal: Transform,
        phase: str,
    ) -> Any:
        carried = context.held_object or context.mounted_tool
        outcome = self.motion.plan_and_execute(goal, phase, carried)
        if not getattr(outcome, 'success', False):
            code = getattr(outcome, 'code', 'failed')
            event = 'collision' if code == 'collision' else 'failed'
            raise SkillError(
                event, getattr(outcome, 'message', 'motion failed'),
            )
        context.last_verified_pose = goal
        work = context.geometry.safe_poses['work']
        context.at_work = (
            _distance(goal, work) <= context.geometry.position_tolerance
            and _angle(goal, work) <= context.geometry.orientation_tolerance_rad
        )
        return outcome

    def _command(
        self,
        context: SkillContext,
        op: str,
        object_id: str = '',
        *,
        target: Transform | None = None,
        target_name: str = '',
        event: str = 'failed',
    ) -> dict[str, Any]:
        try:
            state = self.supervisor.command(
                op, object_id, target=target, target_name=target_name,
            )
        except Exception as error:
            raise SkillError(event, str(error)) from error
        if isinstance(state, dict):
            context.update_from_state(state)
        return state

    # ------------------------------------------------------------------ geometry
    def _live_pose(self, context: SkillContext, object_id: str) -> Transform:
        state = self._state()
        if object_id == 'cap':
            pose = _state_pose((state.get('cap') or {}).get('pose'), 'cap')
            if pose is not None:
                return pose
        if object_id.startswith('screw_'):
            pose = _state_pose(
                (state.get('screws') or {}).get(object_id, {}).get('pose'),
                object_id,
            )
            if pose is not None:
                return pose
        if object_id in context.spec.tool_names:
            pose = _state_pose(
                (state.get('objects') or {}).get(f'grant_{object_id}'),
                object_id,
            )
            if pose is not None:
                return pose
            pose = _state_pose(
                (state.get('tools') or {}).get(object_id, {}).get('pose'),
                object_id,
            )
            if pose is not None:
                return pose
        raise SkillError('failed', f'no live pose for {object_id!r}')

    def _object_contact_goal(
        self, context: SkillContext, object_id: str,
    ) -> Transform:
        if object_id in context.spec.tool_names:
            return tcp_goal(
                self._live_pose(context, object_id),
                context.geometry.tool_tcp_to_nodes[object_id],
            )
        if object_id == 'cap':
            goal = tcp_goal(
                self._live_pose(context, object_id),
                context.geometry.cap_tcp_to_node,
            )
            # The suction cup is rotationally symmetric.  A 180-degree wrist
            # yaw offset keeps the cap grasp on the branch inside the hard
            # joint-7 corridor; the measured relative pose makes the later
            # placement use the same offset.
            return goal.compose_relaxed(Transform.from_xyz_rpy(
                [0.0, 0.0, 0.0], [0.0, 0.0, 180.0],
                parent='tcp', child='cap_grasp_yaw',
            ))
        if object_id.startswith('screw_'):
            return tcp_goal(
                self._live_pose(context, object_id),
                context.geometry.screw_tcp_to_node,
            )
        raise SkillError('failed', f'object {object_id!r} cannot be approached')

    def _destination_goal(
        self,
        context: SkillContext,
        object_id: str,
        target: str,
    ) -> Transform:
        if target in ('', 'work'):
            return context.geometry.safe_poses['work']
        if target == 'hatch_seat':
            seat = world_target(context.spec, context.geometry, 'cap_seat')
            relative = context.cap_relative or context.geometry.cap_tcp_to_node
            return tcp_goal(seat, relative)
        if target.startswith('hole_'):
            return tcp_goal(
                world_target(context.spec, context.geometry, f'hole:{target}'),
                context.geometry.screw_tcp_to_node,
            )
        if target.endswith('_socket'):
            tool = target[: -len('_socket')]
            return tcp_goal(
                world_target(context.spec, context.geometry, f'tool:{tool}'),
                context.geometry.tool_tcp_to_nodes[tool],
            )
        if target in context.spec.sealant_sides:
            point = sealant_point(
                context.geometry, context.spec, target, 0.0,
            )
            return tcp_goal(
                point, context.geometry.tool_work_tips['sealant_tool'],
            )
        raise SkillError('failed', f'unknown target {target!r}')

    def _contact_goal(
        self,
        context: SkillContext,
        object_id: str,
        target: str,
    ) -> Transform:
        if target in context.spec.sealant_sides:
            return self._destination_goal(context, object_id, target)
        if target and target != 'work':
            return self._destination_goal(context, object_id, target)
        return self._object_contact_goal(context, object_id)

    def _signature(self, node: PlanNode) -> tuple[str, str]:
        return (node.object, node.target)

    def _ensure_approach(
        self,
        context: SkillContext,
        node: PlanNode,
        *,
        gap: float | None = None,
    ) -> Transform:
        signature = self._signature(node)
        if context.approach_matches(signature):
            return context.approach_pose  # type: ignore[return-value]
        goal = self._contact_goal(context, node.object, node.target)
        safe = _above(
            goal,
            context.geometry.approach_gap if gap is None else gap,
        )
        self._move(context, safe, 'approach')
        context.mark_approach(safe, signature)
        return safe

    def _ensure_contact(
        self,
        context: SkillContext,
        node: PlanNode,
    ) -> Transform:
        self._ensure_approach(context, node)
        if context.phase == 'contact' and context.contact_pose is not None:
            return context.contact_pose
        goal = self._contact_goal(context, node.object, node.target)
        self._move(context, goal, 'lower')
        context.mark_contact(goal)
        return goal

    def _lift_to_approach(
        self,
        context: SkillContext,
        *,
        label: str = 'lift',
    ) -> None:
        if context.phase == 'lifted':
            return
        if context.approach_pose is None:
            raise SkillError('failed', f'{label}: saved approach pose is unknown')
        self._move(context, context.approach_pose, 'retreat')
        context.mark_lifted()

    def _adapter(self, context: SkillContext) -> Any:
        return context.scene_adapter or self.scene_adapter

    # ------------------------------------------------------------------ watch
    def _detect(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        # An axial withdrawal is mandatory before leaving a contact.
        if context.phase == 'contact' and context.approach_pose is not None:
            self._move(context, context.approach_pose, 'retreat')
            context.mark_lifted()
        work = context.geometry.safe_poses['work']
        if not (context.at_work and context.phase == 'free'):
            phase = 'transfer_lin' if context.phase != 'free' else 'transfer'
            self._move(context, work, phase)
        context.reset_phase()
        context.at_work = True
        if context.capture_frame('top') is None:
            return SkillOutcome('failed', {'reason': 'no fresh top camera frame'})
        if node.object in context.close_inspection_objects:
            if context.capture_frame('hand') is None:
                return SkillOutcome(
                    'failed', {'reason': 'no fresh close-inspection frame'},
                )
        found = self._object_present(context, node.object)
        if not found:
            return SkillOutcome('not_found', {'object': node.object})
        pose = None
        try:
            pose = self._live_pose(context, node.object)
            context.last_verified_pose = pose
        except Exception:
            pose = None
        return SkillOutcome('found', {
            'object': node.object,
            'tcp': list(pose.xyz) if pose is not None else None,
            'close_inspection': node.object in context.close_inspection_objects,
        })

    def _object_present(self, context: SkillContext, object_id: str) -> bool:
        effective = object_id or ''
        if effective in context.spec.tool_names:
            return True
        if effective == 'cap':
            if context.spec.cap_state == 'absent':
                return False
            state = (self._state().get('cap') or {}).get('state', 'free')
            return state != 'absent'
        if effective.startswith('screw_'):
            if not any(screw.name == effective for screw in context.spec.screws):
                return False
            state = (
                (self._state().get('screws') or {}).get(effective, {}).get(
                    'state', 'tray',
                )
            )
            return state != 'delivered'
        try:
            self.catalog.destination_kind(effective, context.spec)
            return True
        except Exception:
            return False

    def _capture(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        stamp = context.capture_frame(node.object)
        if stamp is None:
            return SkillOutcome('failed', {'camera': node.object})
        return SkillOutcome('success', {'camera': node.object, 'stamp': stamp})

    # ------------------------------------------------------------------ phases
    def _approach_skill(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        safe = self._ensure_approach(context, node)
        return SkillOutcome('success', {
            'object': node.object, 'target': node.target,
            'approach_height': context.geometry.approach_gap,
            'tcp': list(safe.xyz),
        })

    def _lower_skill(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        goal = self._ensure_contact(context, node)
        return SkillOutcome('success', {
            'object': node.object, 'target': node.target,
            'tcp': list(goal.xyz),
        })

    def _lift(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        if context.phase == 'lifted':
            return SkillOutcome('success', {'reused': True})
        self._ensure_approach(context, node)
        goal = context.approach_pose
        self._lift_to_approach(context)
        return SkillOutcome('success', {
            'reused': False,
            'tcp': list(goal.xyz) if goal is not None else None,
        })

    # ------------------------------------------------------------------ grab
    def _grab(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        obj = node.object
        unavailable = self._refresh_live_ownership(context)
        if unavailable is not None:
            return SkillOutcome('grasp_failed', {'reason': unavailable})
        if obj in context.spec.tool_names:
            return self._grab_tool(node, context)
        if obj == 'cap':
            if context.mounted_tool != 'vacuum':
                return SkillOutcome('grasp_failed', {
                    'reason': 'required tool vacuum is not mounted',
                    'required_tool': 'vacuum',
                    'mounted_tool': context.mounted_tool,
                })
            if context.held_object is not None:
                return SkillOutcome('grasp_failed', {
                    'reason': f'hand is not free: {context.held_object!r} is held',
                    'held_object': context.held_object,
                })
            return self._grab_part(node, context, 'grab_cap', 'vacuum')
        if obj.startswith('screw_'):
            if context.mounted_tool != 'magnetic_tool':
                return SkillOutcome('grasp_failed', {
                    'reason': 'required tool magnetic_tool is not mounted',
                    'required_tool': 'magnetic_tool',
                    'mounted_tool': context.mounted_tool,
                })
            if context.held_object is not None:
                return SkillOutcome('grasp_failed', {
                    'reason': f'hand is not free: {context.held_object!r} is held',
                    'held_object': context.held_object,
                })
            return self._grab_part(
                node, context, 'grab_screw', 'magnetic_tool',
            )
        return SkillOutcome('failed', {'reason': f'{obj!r} cannot be grasped'})

    def _grab_tool(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        tool = node.object
        if context.mounted_tool == tool:
            return SkillOutcome('success', {'reused': True, 'tool': tool})
        adapter = self._adapter(context)
        prepare = getattr(adapter, 'prepare_grab_tool', None)
        self._ensure_approach(context, node)
        if prepare is not None:
            prepare(tool)
        try:
            self._ensure_contact(context, node)
        except Exception:
            restore = getattr(adapter, 'restore_tool', None)
            if prepare is not None and restore is not None:
                restore(tool)
            raise
        try:
            self._command(
                context, 'attach_tool', tool, event='grasp_failed',
            )
        except SkillError as error:
            context.ownership_unknown = True
            restore = getattr(adapter, 'restore_tool', None)
            if restore is not None:
                try:
                    restore(tool)
                except Exception:
                    pass
            return SkillOutcome('grasp_failed', {'reason': str(error)})
        context.mounted_tool = tool
        context.held_object = None
        context.ownership_unknown = False
        if adapter is not None:
            try:
                adapter.attach_tool(tool)
            except Exception as error:
                return SkillOutcome('failed', {'reason': f'planning scene: {error}'})
        self._lift_to_approach(context)
        pose = context.approach_pose
        return SkillOutcome('success', {
            'tool': tool,
            'tcp': list(pose.xyz) if pose is not None else None,
        })

    def _grab_part(
        self,
        node: PlanNode,
        context: SkillContext,
        command: str,
        tool: str,
    ) -> SkillOutcome:
        adapter = self._adapter(context)
        if node.object == 'cap':
            prepare = getattr(adapter, 'prepare_grab_cap', None)
        else:
            prepare = getattr(adapter, 'prepare_grab_screw', None)
        self._ensure_approach(context, node)
        if prepare is not None:
            prepare(node.object)
        try:
            contact = self._ensure_contact(context, node)
        except Exception:
            restore = getattr(adapter, 'restore_part', None)
            if prepare is not None and restore is not None:
                restore(node.object)
            raise
        try:
            self._command(context, command, node.object, event='grasp_failed')
        except SkillError as error:
            context.ownership_unknown = True
            restore = getattr(adapter, 'restore_part', None)
            if restore is not None:
                try:
                    restore(node.object)
                except Exception:
                    pass
            return SkillOutcome('grasp_failed', {'reason': str(error)})
        context.held_object = node.object
        context.mounted_tool = tool
        context.ownership_unknown = False
        if node.object == 'cap':
            cap_pose = self._live_pose(context, node.object)
            context.cap_relative = contact.inverse().compose_relaxed(cap_pose)
        if adapter is not None:
            try:
                if node.object == 'cap':
                    adapter.attach_cap()
                else:
                    adapter.attach_screw(node.object)
            except Exception as error:
                return SkillOutcome('failed', {'reason': f'planning scene: {error}'})
        self._lift_to_approach(context)
        return SkillOutcome('success', {
            'object': node.object, 'tcp': list(contact.xyz),
        })

    # ------------------------------------------------------------------ transfer
    def _transfer(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        obj = node.object
        target = node.target
        unavailable = self._refresh_live_ownership(context)
        if unavailable is not None:
            return SkillOutcome('failed', {'reason': unavailable})
        if obj in context.spec.tool_names:
            if context.mounted_tool != obj:
                return SkillOutcome('failed', {
                    'reason': f'{obj!r} is not mounted',
                })
        elif obj == 'cap':
            if context.held_object != 'cap':
                return SkillOutcome('failed', {'reason': 'hatch is not held'})
        elif obj.startswith('screw_'):
            if context.held_object != obj:
                return SkillOutcome('failed', {'reason': f'{obj!r} is not held'})
        else:
            return SkillOutcome('failed', {'reason': f'{obj!r} cannot be transferred'})
        goal = self._destination_goal(context, obj, target)
        safe = _above(goal, context.geometry.transfer_gap)
        phase = 'transfer' if obj in context.spec.tool_names else 'transfer_lin'
        self._move(context, safe, phase)
        context.transfer_target = target
        context.mark_approach(safe, (obj, target))
        return SkillOutcome('success', {
            'object': obj, 'target': target,
            'approach_height': context.geometry.transfer_gap,
            'tcp': list(safe.xyz),
        })

    def _move_to_storage(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        if context.held_object is None:
            return SkillOutcome('failed', {'reason': 'nothing is held'})
        transferred = self._transfer(node, context)
        if transferred.event != 'success':
            return transferred
        lowered = self._lower_skill(node, context)
        if lowered.event != 'success':
            return lowered
        return SkillOutcome('success', {
            'object': node.object, 'target': node.target,
            'tcp': list(context.contact_pose.xyz) if context.contact_pose else None,
        })

    def _align(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        if context.held_object is None and context.mounted_tool is None:
            return SkillOutcome('failed', {'reason': 'nothing is held or mounted'})
        if context.approach_pose is not None:
            self._move(context, context.approach_pose, 'align')
            context.phase = 'approach'
            return SkillOutcome('success', {'reused': True})
        safe = self._ensure_approach(context, node)
        return SkillOutcome('success', {'tcp': list(safe.xyz)})

    # ------------------------------------------------------------------ release
    def _release(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        obj = node.object
        target = node.target
        unavailable = self._refresh_live_ownership(context)
        if unavailable is not None:
            return SkillOutcome('failed', {'reason': unavailable})
        if obj in context.spec.tool_names:
            return self._release_tool(node, context)
        if obj == 'cap':
            return self._release_cap(node, context)
        if obj.startswith('screw_'):
            return self._handoff_screw(node, context)
        return SkillOutcome('failed', {'reason': f'{obj!r} cannot be released'})

    def _release_tool(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        tool = node.object
        socket = f'{tool}_socket'
        if context.mounted_tool != tool:
            return SkillOutcome('failed', {
                'reason': f'required tool {tool} is not mounted',
                'required_tool': tool,
                'mounted_tool': context.mounted_tool,
                'moved': False,
            })
        if node.target != socket:
            # A bare release(vacuum) from work must not move the robot or
            # detach the cup.  Only an explicit socket target is valid.
            return SkillOutcome('failed', {
                'reason': (
                    f'release({tool}) requires the explicit socket target '
                    f'{socket!r}; no motion'
                ),
                'moved': False,
            })
        if context.transfer_target != socket:
            return SkillOutcome('failed', {
                'reason': (
                    f'release({tool},{socket}) requires an explicit preceding '
                    'transfer to that socket'
                ),
                'moved': False,
            })
        adapter = self._adapter(context)
        prepare = getattr(adapter, 'prepare_dock_tool', None)
        if prepare is not None:
            prepare(tool)
        if not context.approach_matches((tool, socket)):
            goal = self._destination_goal(context, tool, socket)
            safe = _above(goal, context.geometry.approach_gap)
            self._move(context, safe, 'transfer')
            context.mark_approach(safe, (tool, socket))
        if context.phase != 'contact':
            goal = self._destination_goal(context, tool, socket)
            self._move(context, goal, 'lower')
            context.mark_contact(goal)
        try:
            self._command(context, 'dock_tool', tool)
        except SkillError as error:
            return SkillOutcome('failed', {'reason': str(error)})
        context.mounted_tool = None
        adapter = self._adapter(context)
        if adapter is not None:
            try:
                adapter.dock_tool(tool)
            except Exception as error:
                return SkillOutcome('failed', {'reason': f'planning scene: {error}'})
        self._lift_to_approach(context)
        return SkillOutcome('success', {'tool': tool, 'target': socket})

    def _release_cap(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        if context.held_object != 'cap':
            return SkillOutcome('failed', {
                'reason': 'hatch is not held',
                'held_object': context.held_object,
            })
        target = node.target or 'hatch_seat'
        self._ensure_approach(context, node)
        adapter = self._adapter(context)
        prepare = getattr(adapter, 'prepare_place_cap', None)
        if prepare is not None:
            prepare()
        if context.phase != 'contact':
            goal = self._destination_goal(context, 'cap', target)
            self._move(context, goal, 'lower')
            context.mark_contact(goal)
        seat = world_target(context.spec, context.geometry, 'cap_seat')
        try:
            self._command(context, 'place_cap', 'cap', target=seat)
        except SkillError as error:
            return SkillOutcome('failed', {'reason': str(error)})
        context.held_object = None
        context.cap_relative = None
        adapter = self._adapter(context)
        if adapter is not None:
            try:
                adapter.place_cap(seat)
            except Exception as error:
                return SkillOutcome('failed', {'reason': f'planning scene: {error}'})
        self._lift_to_approach(context)
        return SkillOutcome('success', {'object': 'cap', 'target': target})

    def _dock_tool(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        unavailable = self._refresh_live_ownership(context)
        if unavailable is not None:
            return SkillOutcome('failed', {'reason': unavailable})
        if node.object not in context.spec.tool_names:
            return SkillOutcome('failed', {'reason': f'{node.object!r} is not a tool'})
        socket = f'{node.object}_socket'
        if node.target != socket:
            return SkillOutcome('failed', {
                'reason': f'dock_tool requires target {socket!r}',
            })
        return self._release_tool(node, context)

    def _handoff_screw(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        screw = node.object
        hole = node.target
        unavailable = self._refresh_live_ownership(context)
        if unavailable is not None:
            return SkillOutcome('grasp_failed', {'reason': unavailable})
        if context.mounted_tool != 'magnetic_tool':
            return SkillOutcome('grasp_failed', {
                'reason': 'required tool magnetic_tool is not mounted',
                'required_tool': 'magnetic_tool',
                'mounted_tool': context.mounted_tool,
            })
        if context.held_object != screw:
            return SkillOutcome('grasp_failed', {
                'reason': f'{screw!r} is not held',
                'held_object': context.held_object,
            })
        self._ensure_approach(context, node)
        adapter = self._adapter(context)
        prepare = getattr(adapter, 'prepare_handoff_screw', None)
        if prepare is not None:
            prepare(screw)
        if context.phase != 'contact':
            goal = self._destination_goal(context, screw, hole)
            self._move(context, goal, 'lower')
            context.mark_contact(goal)
        target = world_target(context.spec, context.geometry, f'hole:{hole}')
        try:
            self._command(
                context, 'handoff_screw', screw,
                target=target, target_name=hole, event='grasp_failed',
            )
        except SkillError as error:
            return SkillOutcome('grasp_failed', {'reason': str(error)})
        context.held_object = None
        adapter = self._adapter(context)
        if adapter is not None:
            try:
                adapter.handoff_screw(screw)
            except Exception as error:
                return SkillOutcome('failed', {'reason': f'planning scene: {error}'})
        self._lift_to_approach(context)
        return SkillOutcome('success', {
            'object': screw, 'target': hole,
            'installed': False, 'tightened': False,
        })

    # ------------------------------------------------------------------ misc
    def _retreat(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        del node
        if context.phase == 'contact' and context.approach_pose is not None:
            self._move(context, context.approach_pose, 'retreat')
            context.mark_lifted()
        work = context.geometry.safe_poses['work']
        self._move(context, work, 'transfer')
        context.reset_phase()
        return SkillOutcome('success', {'tcp': list(work.xyz)})

    def _dispense(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        unavailable = self._refresh_live_ownership(context)
        if unavailable is not None:
            return SkillOutcome('failed', {'reason': unavailable})
        if context.mounted_tool != 'sealant_tool':
            return SkillOutcome('failed', {
                'reason': 'required tool sealant_tool is not mounted',
                'required_tool': 'sealant_tool',
                'mounted_tool': context.mounted_tool,
            })
        side = node.object or node.target
        start_fraction = float(node.params.get('start_fraction', 0.0))
        end_fraction = float(node.params.get('end_fraction', 1.0))
        start = sealant_point(
            context.geometry, context.spec, side, start_fraction,
        )
        end = sealant_point(
            context.geometry, context.spec, side, end_fraction,
        )
        tip = context.geometry.tool_work_tips['sealant_tool']
        start_goal = tcp_goal(start, tip)
        end_goal = tcp_goal(end, tip)
        signature = (side, f'{start_fraction:.3f}:{end_fraction:.3f}')
        if not context.approach_matches(signature):
            safe = _above(start_goal, context.geometry.approach_gap)
            self._move(context, safe, 'approach')
            context.mark_approach(safe, signature)
        if context.phase != 'contact':
            self._move(context, start_goal, 'lower')
            context.mark_contact(start_goal)
        self._move(context, end_goal, 'dispense')
        context.mark_contact(end_goal)
        self._lift_to_approach(context)
        return SkillOutcome('success', {
            'side': side,
            'start_fraction': start_fraction,
            'end_fraction': end_fraction,
        })

    def _stop(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        del node
        try:
            self.motion.stop('graph stop node')
            # A stop node is a graph decision, not a terminal safety latch:
            # a verified inspect_state may deliberately retry afterwards.
            reset = getattr(self.motion, 'reset_stop', None)
            if reset is not None:
                reset()
        except Exception as error:
            return SkillOutcome('failed', {'reason': str(error)})
        return SkillOutcome('success', {
            'tool': context.mounted_tool,
            'held': context.held_object,
        })

    def _wait_scene_clear(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        del node
        if context.scene_is_clear():
            return SkillOutcome('clear')
        return SkillOutcome('timeout', {'reason': 'scene did not become clear'})

    def _inspect_state(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        state = self._state()
        obj = node.object
        if not state:
            context.ownership_unknown = True
            return SkillOutcome('unknown', {'reason': 'supervisor state unavailable'})
        mounted = state.get('mounted_tool')
        held = state.get('held_object')
        if held == obj:
            context.mounted_tool = mounted if isinstance(mounted, str) else context.mounted_tool
            context.held_object = obj
            context.ownership_unknown = False
            return SkillOutcome('holding', {'object': obj})
        if obj == 'cap':
            cap_state = (state.get('cap') or {}).get('state')
            if cap_state == 'installed':
                context.held_object = None
                context.ownership_unknown = False
                return SkillOutcome('placed', {'object': obj})
            if cap_state == 'free':
                context.held_object = None
                context.ownership_unknown = False
                return SkillOutcome('empty', {'object': obj})
            if cap_state == 'held':
                context.held_object = 'cap'
                context.ownership_unknown = False
                return SkillOutcome('holding', {'object': obj})
        if obj.startswith('screw_'):
            screw = (state.get('screws') or {}).get(obj, {})
            screw_state = screw.get('state')
            if screw_state == 'delivered':
                context.held_object = None
                context.ownership_unknown = False
                return SkillOutcome('placed', {'object': obj})
            if screw_state == 'held':
                context.held_object = obj
                context.ownership_unknown = False
                return SkillOutcome('holding', {'object': obj})
            if screw_state == 'tray':
                context.held_object = None
                context.ownership_unknown = False
                return SkillOutcome('empty', {'object': obj})
        if obj in context.spec.tool_names:
            tool_state = (state.get('tools') or {}).get(obj, {}).get('state')
            if tool_state == 'mounted':
                context.mounted_tool = obj
                context.ownership_unknown = False
                return SkillOutcome('holding', {'object': obj})
            if tool_state == 'docked':
                context.mounted_tool = None
                context.ownership_unknown = False
                return SkillOutcome('empty', {'object': obj})
        context.ownership_unknown = True
        return SkillOutcome('unknown', {
            'object': obj, 'mounted_tool': mounted, 'held_object': held,
        })

    def _notify_operator(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        reason = node.target or 'operator intervention required'
        if context.notify(reason, {
            'node': node.id, 'tool': context.mounted_tool,
            'held': context.held_object,
        }):
            return SkillOutcome('success', {'reason': reason})
        return SkillOutcome('failed', {'reason': reason})

    def _verify(self, node: PlanNode, context: SkillContext) -> SkillOutcome:
        state = self._state()
        obj = node.object
        target = node.target
        if not state:
            return SkillOutcome('failed', {'reason': 'supervisor state unavailable'})
        if obj == 'cap':
            cap = state.get('cap') or {}
            if cap.get('state') != 'installed':
                return SkillOutcome('not_ok', {
                    'reason': f'hatch state is {cap.get("state")!r}',
                })
            pose = _state_pose(cap.get('pose'), 'cap')
            if pose is None:
                return SkillOutcome('failed', {'reason': 'hatch pose unavailable'})
            seat = target or 'hatch_seat'
            try:
                expected = world_target(
                    context.spec, context.geometry, 'cap_seat',
                )
            except Exception as error:
                return SkillOutcome('failed', {'reason': str(error)})
            distance = _distance(pose, expected)
            if distance <= context.geometry.position_tolerance:
                return SkillOutcome('ok', {
                    'object': obj, 'target': seat, 'distance': distance,
                })
            return SkillOutcome('not_ok', {
                'object': obj, 'target': seat, 'distance': distance,
            })
        if obj.startswith('screw_'):
            screw = (state.get('screws') or {}).get(obj, {})
            if screw.get('state') == 'delivered' and screw.get('hole') == target:
                return SkillOutcome('ok', {'object': obj, 'target': target})
            return SkillOutcome('not_ok', {
                'object': obj, 'target': target,
                'state': screw.get('state'),
            })
        inspected = self._inspect_state(node, context)
        if inspected.event == 'placed':
            return SkillOutcome('ok', {'object': obj})
        if inspected.event in ('empty', 'holding'):
            return SkillOutcome('not_ok', {'object': obj, 'state': inspected.event})
        return SkillOutcome('failed', {'object': obj})
