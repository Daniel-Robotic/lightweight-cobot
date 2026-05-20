from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, List, Optional, Tuple

from ruamel.yaml import YAML
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.screen import Screen
from textual.widgets import Footer, Static

from cobot.tui import SCREEN_CSS, InputScreen, PickScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent
_CONFIG_PATH = _PROJECT_DIR / "cobot-setting.yaml"

_yaml = YAML()
_yaml.preserve_quotes = True


@dataclass
class _Field:
    key: str               # dot-separated path within the block, e.g. "webots.world"
    question: str
    default: Any
    note: str = ""
    options: Optional[List[str]] = None  # if set - PickScreen, else - InputScreen

    def label(self) -> str:
        return self.key.split(".")[-1]


@dataclass
class _Block:
    yaml_key: str          # top-level key in cobot-setting.yaml
    title: str             # shown in "Configure <title>?" prompt
    fields: List[_Field]


_BLOCKS: List[_Block] = [
    _Block(
        yaml_key="foxglove",
        title="Foxglove bridge",
        fields=[
            _Field("enabled", "Enable Foxglove bridge?", "true",
                   note="Start foxglove_bridge alongside the robot node",
                   options=["true", "false"]),
            _Field("port", "WebSocket port:", "8765",
                   note="Port Foxglove Studio connects to (default 8765)"),
            _Field("address", "Listen address:", "0.0.0.0",
                   note="0.0.0.0 = all interfaces, 127.0.0.1 = localhost only",
                   options=["0.0.0.0", "127.0.0.1"]),
            _Field("use_sim_time", "Use simulation time (/clock)?", "false",
                   note="Subscribe to /clock instead of using wall time",
                   options=["false", "true"]),
            _Field("debug", "Enable verbose bridge logging?", "false",
                   options=["false", "true"]),
            _Field("num_threads", "Executor threads (0 = auto):", "0"),
        ],
    ),
    _Block(
        yaml_key="planning",
        title="MoveIt planning",
        fields=[
            _Field("pose_link", "TCP link name:", "tcp",
                   note="Link used as the end-effector for Cartesian goals (defined in URDF/SRDF)"),
            _Field("planning_group", "Planning group:", "iiwa_arm",
                   note="MoveIt planning group as defined in the SRDF"),
            _Field("default_frame", "Default reference frame:", "base_link"),
            _Field("default_planner", "Default planner:", "ompl",
                   options=["ompl", "pilz_industrial_motion_planner", "chomp"]),
            _Field("planning_attempts", "Planning attempts:", "3"),
        ],
    ),
    _Block(
        yaml_key="digital_twin",
        title="Digital twin (Webots / RViz)",
        fields=[
            _Field("webots.transform", "Robot transform in Webots scene (x y z, metres):", "-0.25 0 0.79"),
            _Field("webots.rotation", "Robot rotation in Webots scene (ax ay az angle):", "0 0 1 0"),
            _Field("webots.controller_timer", "Webots controller step timer (ms):", "50"),
        ],
    ),
    _Block(
        yaml_key="robot",
        title="Robot connection",
        fields=[
            _Field("name", "Robot model name:", "iiwa7"),
            _Field("ip", "Robot IP address:", "192.170.10.2",
                   note="IP of the KUKA controller on the FRI network interface"),
            _Field("port", "FRI port:", "30200"),
            _Field("command_mode", "Command mode:", "position",
                   note="position = joint position control, torque = joint torque control",
                   options=["position", "torque"]),
            _Field("fri_cycle_ms", "FRI cycle time (ms):", "10",
                   note="5 ms = 200 Hz, 10 ms = 100 Hz",
                   options=["10", "5"]),
            _Field("active_controller", "Active ROS controller:", "jtc",
                   note="jtc = JointTrajectoryController (MoveIt), forward = ForwardCommandController",
                   options=["jtc", "forward"]),
            _Field("joint_position_tau", "Position EMA filter τ (s):", "0.04",
                   note="Smooths position commands before sending to FRI"),
            _Field("joint_velocity_tau", "Velocity EMA filter τ (s):", "0.01",
                   note="Removes spikes from finite-difference velocity estimation"),
        ],
    ),
]


def _coerce(value: str, original: Any) -> Any:
    """Try to preserve the original YAML scalar type."""
    if isinstance(original, bool):
        return value.lower() == "true"
    if isinstance(original, int):
        try:
            return int(value)
        except ValueError:
            return value
    if isinstance(original, float):
        try:
            return float(value)
        except ValueError:
            return value
    return value


def _get_nested(mapping: Any, path: str) -> Any:
    keys = path.split(".")
    cur = mapping
    for k in keys:
        if cur is None or k not in cur:
            return None
        cur = cur[k]
    return cur


def _set_nested(mapping: Any, path: str, value: Any) -> None:
    keys = path.split(".")
    cur = mapping
    for k in keys[:-1]:
        cur = cur[k]
    original = cur[keys[-1]]
    cur[keys[-1]] = _coerce(value, original)


class _SavedScreen(Screen[None]):
    BINDINGS = [Binding("enter,escape", "close", "Close")]

    def compose(self) -> ComposeResult:
        yield Static("Done", id="step")
        yield Static(f"Configuration saved to {_CONFIG_PATH.name}", id="question")
        yield Static("Press Enter to close.", id="note")
        yield Footer()

    def action_close(self) -> None:
        self.dismiss(None)


class _Wizard(App[None]):
    CSS = SCREEN_CSS

    def __init__(self, data: Any):
        super().__init__()
        self._data = data
        self._blocks = list(_BLOCKS)  # copy so we can pop
        self._block_idx = 0
        self._field_idx = 0
        self._current_block: Optional[_Block] = None
        self._pending_fields: List[_Field] = []

    def on_mount(self) -> None:
        self._next_block()


    def _next_block(self) -> None:
        if self._block_idx >= len(self._blocks):
            _save_config(self._data)
            self.push_screen(_SavedScreen(), lambda _: self.exit())
            return
        block = self._blocks[self._block_idx]
        total = len(self._blocks)
        step = f"Block {self._block_idx + 1} of {total}"
        self.push_screen(
            PickScreen(
                step,
                f"Configure {block.title}?",
                ["Yes", "No"],
                "Yes",
            ),
            lambda v: self._got_block_choice(v, block),
        )

    def _got_block_choice(self, v: Optional[str], block: _Block) -> None:
        if v is None:
            self.exit()
            return
        self._block_idx += 1
        if v == "Yes":
            self._current_block = block
            self._pending_fields = list(block.fields)
            self._field_idx = 0
            self._next_field()
        else:
            self._next_block()


    def _next_field(self) -> None:
        if not self._pending_fields:
            self._next_block()
            return

        f = self._pending_fields[0]
        block = self._current_block
        total_blocks = len(self._blocks)
        block_num = self._block_idx  # already incremented
        self._field_idx += 1
        field_num = self._field_idx
        total_fields = len(block.fields)

        step = f"Block {block_num} of {total_blocks}  ·  Field {field_num} of {total_fields}"

        # Resolve current value from loaded YAML as the pre-filled default
        yaml_val = _get_nested(self._data[block.yaml_key], f.key)
        current = str(yaml_val) if yaml_val is not None else f.default

        if f.options:
            # Make the current value the default selection
            default_opt = current if current in f.options else f.options[0]
            screen = PickScreen(step, f.question, f.options, default_opt, note=f.note)
        else:
            screen = InputScreen(step, f.question, current, note=f.note)

        self.push_screen(screen, lambda v, _f=f: self._got_field(v, _f))

    def _got_field(self, v: Optional[str], f: _Field) -> None:
        if v is None:
            self.exit()
            return
        block = self._current_block
        _set_nested(self._data[block.yaml_key], f.key, v)
        self._pending_fields.pop(0)
        self._next_field()



def _load_config() -> Any:
    with open(_CONFIG_PATH, "r", encoding="utf-8") as fh:
        return _yaml.load(fh)


def _save_config(data: Any) -> None:
    with open(_CONFIG_PATH, "w", encoding="utf-8") as fh:
        _yaml.dump(data, fh)


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("robot-setup", help="Configure cobot-setting.yaml interactively")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    if not _CONFIG_PATH.exists():
        from rich.console import Console
        Console().print(f"[red]Config not found:[/red] {_CONFIG_PATH}")
        sys.exit(1)

    data = _load_config()
    _Wizard(data).run()
