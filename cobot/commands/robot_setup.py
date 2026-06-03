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

_TOOLS_YAML = _PROJECT_DIR / "src" / "iiwa_config" / "config" / "tools.yaml"
_TOOL_ACTIVE_XACRO = (
    _PROJECT_DIR / "src" / "iiwa_description" / "urdf" / "tools" / "tool_active.xacro"
)
_SRDF_PATH = _PROJECT_DIR / "src" / "iiwa_config" / "config" / "moveit" / "iiwa7.srdf"

# Use ruamel.yaml instead of PyYAML so comments and formatting in the config file are preserved.
# Используем ruamel.yaml вместо PyYAML, чтобы комментарии и форматирование в конфиге сохранялись.
_yaml = YAML()
_yaml.preserve_quotes = True


# One question inside a configuration block.
# A field can either show a pick list (options) or a free-text input (no options).
# Один вопрос внутри блока конфигурации.
# Поле может показывать список вариантов (options) или поле для ввода текста (без options).
@dataclass
class _Field:
    key: str               # dot-separated path within the block, e.g. "webots.world"
    question: str
    default: Any
    note: str = ""
    options: Optional[List[str]] = None  # if set - PickScreen, else - InputScreen

    def label(self) -> str:
        return self.key.split(".")[-1]


# A group of related fields shown together under one "Configure X?" question.
# Группа связанных полей, показываемая вместе под одним вопросом "Настроить X?".
@dataclass
class _Block:
    yaml_key: str          # top-level key in cobot-setting.yaml
    title: str             # shown in "Configure <title>?" prompt
    fields: List[_Field]


def _load_tools_registry() -> dict:
    """Читает tools.yaml и возвращает словарь инструментов."""
    if not _TOOLS_YAML.exists():
        return {}
    _y = YAML()
    with open(_TOOLS_YAML, encoding="utf-8") as f:
        data = _y.load(f)
    return dict(data.get("tools", {}))


def _build_tool_block() -> Optional[_Block]:
    """Строит блок выбора инструмента из реестра tools.yaml.
    Возвращает None если реестр недоступен."""
    registry = _load_tools_registry()
    if not registry:
        return None
    options = list(registry.keys())
    labels = "  |  ".join(
        f"{name}: {registry[name].get('label', '')}" for name in options
    )
    return _Block(
        yaml_key="tool",
        title="Tool / End-effector",
        fields=[
            _Field(
                "active",
                "Выберите активный инструмент:",
                options[0],
                note=labels,
                options=options,
            ),
        ],
    )


# All configuration blocks. Each block maps to a top-level key in cobot-setting.yaml.
# Все блоки конфигурации. Каждый блок соответствует ключу верхнего уровня в cobot-setting.yaml.
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
            _Field("joint_position_tau", "Position EMA filter tau (s):", "0.04",
                   note="Smooths position commands before sending to FRI"),
            _Field("joint_velocity_tau", "Velocity EMA filter tau (s):", "0.01",
                   note="Removes spikes from finite-difference velocity estimation"),
        ],
    ),
]


# Try to keep the original YAML type (bool, int, float) when saving a value back.
# Trying to preserve type prevents "true" from becoming a plain string in the YAML file.
# Пытаемся сохранить исходный тип YAML (bool, int, float) при записи значения обратно.
# Сохранение типа предотвращает превращение "true" в обычную строку в YAML-файле.
def _coerce(value: str, original: Any) -> Any:
    """Convert a string value to match the type of the original YAML value (bool, int, float, str).
    Преобразует строковое значение к типу исходного значения YAML (bool, int, float, str).
    """
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


# Read a value from a nested YAML mapping using a dot-separated key like "webots.transform".
# Читаем значение из вложенного YAML-словаря по ключу с точками, например "webots.transform".
def _get_nested(mapping: Any, path: str) -> Any:
    """Return the value at a dot-separated path inside a nested YAML mapping, or None if missing.
    Возвращает значение по пути с точками внутри вложенного YAML-словаря, или None если отсутствует.
    """
    keys = path.split(".")
    cur = mapping
    for k in keys:
        if cur is None or k not in cur:
            return None
        cur = cur[k]
    return cur


# Write a value into a nested YAML mapping using a dot-separated key.
# Записываем значение в вложенный YAML-словарь по ключу с точками.
def _set_nested(mapping: Any, path: str, value: Any) -> None:
    """Set the value at a dot-separated path inside a nested YAML mapping, coercing type to match.
    Устанавливает значение по пути с точками во вложенном YAML-словаре, приводя тип к исходному.
    """
    keys = path.split(".")
    cur = mapping
    for k in keys[:-1]:
        cur = cur[k]
    original = cur[keys[-1]]
    cur[keys[-1]] = _coerce(value, original)


# Shown after all blocks have been configured to confirm the file was saved.
# Показывается после настройки всех блоков для подтверждения сохранения файла.
class _SavedScreen(Screen[None]):
    """Confirmation screen shown after all configuration blocks are saved. Press Enter to close.
    Экран подтверждения, показываемый после сохранения всех блоков конфигурации. Enter для закрытия.
    """
    BINDINGS = [Binding("enter,escape", "close", "Close")]

    def compose(self) -> ComposeResult:
        yield Static("Done", id="step")
        yield Static(f"Configuration saved to {_CONFIG_PATH.name}", id="question")
        yield Static("Press Enter to close.", id="note")
        yield Footer()

    def action_close(self) -> None:
        self.dismiss(None)


# The main configuration wizard. Goes through each block in order.
# For each block it first asks "Configure X?" then steps through all its fields.
# Главный мастер конфигурации. Проходит по каждому блоку по порядку.
# Для каждого блока сначала спрашивает "Настроить X?" а затем проходит по всем его полям.
class _Wizard(App[None]):
    """Configuration wizard that iterates over all _BLOCKS. For each block it asks
    "Configure X?" and if confirmed steps through every field with PickScreen or InputScreen.
    Saves to cobot-setting.yaml when all blocks are done and shows _SavedScreen.
    Мастер конфигурации, проходящий по всем _BLOCKS. Для каждого блока спрашивает
    "Настроить X?" и при подтверждении проходит по всем полям через PickScreen или InputScreen.
    Сохраняет в cobot-setting.yaml по завершении и показывает _SavedScreen.
    """
    CSS = SCREEN_CSS

    def __init__(self, data: Any, extra_blocks: Optional[List[_Block]] = None):
        super().__init__()
        self._data = data
        self._blocks = list(extra_blocks or []) + list(_BLOCKS)
        self._block_idx = 0
        self._field_idx = 0
        self._current_block: Optional[_Block] = None
        self._pending_fields: List[_Field] = []
        self.did_save = False

    def on_mount(self) -> None:
        self._next_block()

    def _next_block(self) -> None:
        if self._block_idx >= len(self._blocks):
            # All blocks done - save and show the confirmation screen.
            # Все блоки пройдены - сохраняем и показываем экран подтверждения.
            _save_config(self._data)
            self.did_save = True
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
            # Skip all fields in this block and jump to the next block.
            # Пропускаем все поля этого блока и переходим к следующему.
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

        step = f"Block {block_num} of {total_blocks}  -  Field {field_num} of {total_fields}"

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
        # Remove the field we just handled and move on to the next one.
        # Удаляем только что обработанное поле и переходим к следующему.
        self._pending_fields.pop(0)
        self._next_field()


# Load the config file preserving all comments and key order.
# Загружаем конфиг-файл, сохраняя все комментарии и порядок ключей.
def _load_config() -> Any:
    """Load cobot-setting.yaml with ruamel.yaml, preserving comments and key order.
    Загружает cobot-setting.yaml с помощью ruamel.yaml, сохраняя комментарии и порядок ключей.
    """
    with open(_CONFIG_PATH, "r", encoding="utf-8") as fh:
        return _yaml.load(fh)


# Write the modified config back to disk preserving comments and formatting.
# Записываем изменённый конфиг обратно на диск, сохраняя комментарии и форматирование.
def _save_config(data: Any) -> None:
    """Write the modified YAML data back to cobot-setting.yaml, preserving comments.
    Записывает изменённые данные YAML обратно в cobot-setting.yaml, сохраняя комментарии.
    """
    with open(_CONFIG_PATH, "w", encoding="utf-8") as fh:
        _yaml.dump(data, fh)


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("robot-setup", help="Configure cobot-setting.yaml interactively")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    from rich.console import Console
    console = Console()

    if not _CONFIG_PATH.exists():
        console.print(f"[red]Config not found:[/red] {_CONFIG_PATH}")
        sys.exit(1)

    data = _load_config()

    # Убеждаемся что секция tool существует в данных (для старых конфигов)
    if "tool" not in data:
        data["tool"] = {"active": "patron"}

    tool_block = _build_tool_block()
    extra = [tool_block] if tool_block else []

    wizard = _Wizard(data, extra_blocks=extra)
    wizard.run()

    if not wizard.did_save:
        return

    # Применяем выбранный инструмент: перезаписываем tool_active.xacro и iiwa7.srdf
    active_tool = str(data["tool"].get("active", "patron"))
    if _TOOLS_YAML.exists():
        try:
            registry = _load_tools_registry()
            if active_tool not in registry:
                raise ValueError(
                    f"Unknown tool '{active_tool}'. Available: {', '.join(registry)}"
                )
            tool_cfg = dict(registry[active_tool])

            import sys as _sys
            _sys.path.insert(0, str(_PROJECT_DIR / "src" / "iiwa_utils"))
            from iiwa_utils.tool_manager import apply_tool
            apply_tool(
                tool_cfg=tool_cfg,
                xacro_out_path=_TOOL_ACTIVE_XACRO,
                srdf_path=_SRDF_PATH,
            )

            # Синхронизируем planning.pose_link с tcp_link выбранного инструмента
            tcp_link = tool_cfg.get("tcp_link", "link_ee")
            if "planning" in data:
                data["planning"]["pose_link"] = tcp_link
                _save_config(data)

            console.print(
                f"[green]✓[/green] Tool [bold]{active_tool}[/bold] applied: "
                f"tool_active.xacro, iiwa7.srdf updated, "
                f"planning.pose_link → [bold]{tcp_link}[/bold]."
            )
        except Exception as exc:
            console.print(f"[red]Tool apply failed:[/red] {exc}")
    else:
        console.print(f"[yellow]tools.yaml not found at {_TOOLS_YAML} — skipping tool apply[/yellow]")
