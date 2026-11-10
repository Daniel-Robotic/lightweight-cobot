"""Execution context shared by the skill registry and the graph engine."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from iiwa_experement.geometry import GeometrySpec, Transform
from iiwa_experement.scenario import ScenarioSpec


@dataclass
class SkillContext:
    spec: ScenarioSpec
    geometry: GeometrySpec
    scene_revision: int = 0
    mounted_tool: str | None = None
    held_object: str | None = None
    ownership_unknown: bool = False
    approach_pose: Transform | None = None
    approach_signature: tuple[str, str] | None = None
    contact_pose: Transform | None = None
    phase: str = 'free'  # free | approach | contact | lifted
    transfer_target: str | None = None
    last_verified_pose: Transform | None = None
    cap_relative: Transform | None = None
    supervisor_state: dict[str, Any] = field(default_factory=dict)
    accepted_revisions: set[int] = field(default_factory=set)
    close_inspection_objects: frozenset[str] = frozenset()

    # Injected sources, replaced in tests.
    frame_provider: Callable[[str], float | None] | None = None
    scene_clear_provider: Callable[[], bool] | None = None
    notifier: Callable[[str, dict[str, Any]], bool] | None = None
    scene_adapter: Any = None

    # ------------------------------------------------------------------ phases
    def mark_approach(self, pose: Transform, signature: tuple[str, str]) -> None:
        self.approach_pose = pose
        self.approach_signature = signature
        self.contact_pose = None
        self.phase = 'approach'

    def mark_contact(self, pose: Transform) -> None:
        self.contact_pose = pose
        self.phase = 'contact'

    def mark_lifted(self) -> None:
        self.contact_pose = None
        self.phase = 'lifted'

    def reset_phase(self) -> None:
        self.approach_pose = None
        self.approach_signature = None
        self.contact_pose = None
        self.phase = 'free'
        self.transfer_target = None

    def approach_matches(self, signature: tuple[str, str]) -> bool:
        return (
            self.approach_pose is not None
            and self.approach_signature == signature
            and self.phase in ('approach', 'contact', 'lifted')
        )

    # ------------------------------------------------------------------ state
    def update_from_state(self, state: dict[str, Any]) -> None:
        if not isinstance(state, dict):
            return
        self.supervisor_state = state
        mounted = state.get('mounted_tool')
        held = state.get('held_object')
        self.mounted_tool = mounted if isinstance(mounted, str) else None
        self.held_object = held if isinstance(held, str) else None
        version = state.get('version')
        if isinstance(version, int):
            self.accepted_revisions.add(self.scene_revision)
        self.ownership_unknown = False

    def capture_frame(self, name: str) -> float | None:
        if self.frame_provider is None:
            return None
        try:
            stamp = self.frame_provider(name)
        except Exception:
            return None
        if stamp is None:
            return None
        try:
            return float(stamp)
        except (TypeError, ValueError):
            return None

    def scene_is_clear(self) -> bool:
        if self.scene_clear_provider is None:
            return True
        try:
            return bool(self.scene_clear_provider())
        except Exception:
            return False

    def notify(self, reason: str, details: dict[str, Any]) -> bool:
        if self.notifier is None:
            return True
        try:
            return bool(self.notifier(reason, details))
        except Exception:
            return False
