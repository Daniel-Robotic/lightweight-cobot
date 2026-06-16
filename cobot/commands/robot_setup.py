from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional

from ruamel.yaml import YAML

from cobot import ui

_PROJECT_DIR = Path(__file__).parent.parent.parent
_CONFIG_PATH = _PROJECT_DIR / "cobot-setting.yaml"

_TOOLS_YAML = _PROJECT_DIR / "src" / "iiwa_config" / "config" / "tools.yaml"
_TOOL_ACTIVE_XACRO = (
    _PROJECT_DIR / "src" / "iiwa_description" / "urdf" / "tools" / "tool_active.xacro"
)
_SRDF_PATH = _PROJECT_DIR / "src" / "iiwa_config" / "config" / "moveit" / "iiwa7.srdf"

# Use ruamel.yaml instead of PyYAML so comments and formatting in the config are preserved.
# Используем ruamel.yaml вместо PyYAML, чтобы комментарии и форматирование в конфиге сохранялись.
_yaml = YAML()
_yaml.preserve_quotes = True


@dataclass
class _Field:
    key: str               # dot-separated path within the block, e.g. "webots.world"
    question: str
    default: Any
    note: str = ""
    options: Optional[List[str]] = None  # if set - select(), else - text()

    def label(self) -> str:
        return self.key.split(".")[-1]


@dataclass
class _Block:
    yaml_key: str          # top-level key in cobot-setting.yaml
    title: str             # shown in "Настроить <title>?" prompt
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
        title="Инструмент / Захват",
        fields=[
            _Field("active", "Выберите активный инструмент:", options[0],
                   note=labels, options=options),
        ],
    )


# All configuration blocks. Each block maps to a top-level key in cobot-setting.yaml.
# Все блоки конфигурации. Каждый блок соответствует ключу верхнего уровня в cobot-setting.yaml.
_BLOCKS: List[_Block] = [
    _Block(
        yaml_key="foxglove",
        title="Foxglove bridge",
        fields=[
            _Field("enabled", "Включить Foxglove bridge?", "true",
                   note="Запускать foxglove_bridge вместе с узлом робота",
                   options=["true", "false"]),
            _Field("port", "Порт WebSocket:", "8765",
                   note="Порт, к которому подключается Foxglove Studio (по умолчанию 8765)"),
            _Field("address", "Адрес прослушивания:", "0.0.0.0",
                   note="0.0.0.0 = все интерфейсы, 127.0.0.1 = только localhost",
                   options=["0.0.0.0", "127.0.0.1"]),
            _Field("use_sim_time", "Использовать симуляционное время (/clock)?", "false",
                   note="Подписываться на /clock вместо системного времени",
                   options=["false", "true"]),
            _Field("debug", "Подробное логирование bridge?", "false",
                   options=["false", "true"]),
            _Field("num_threads", "Потоки executor (0 = авто):", "0"),
        ],
    ),
    _Block(
        yaml_key="web",
        title="Веб-сервер (FastAPI)",
        fields=[
            _Field("enabled", "Включить веб-сервер?", "true",
                   note="Запускать FastAPI-сервер для HTTP/WebSocket-управления",
                   options=["true", "false"]),
            _Field("host", "Адрес прослушивания:", "0.0.0.0",
                   note="0.0.0.0 = все интерфейсы, 127.0.0.1 = только localhost",
                   options=["0.0.0.0", "127.0.0.1"]),
            _Field("port", "Порт HTTP:", "8007",
                   note="Порт FastAPI-сервера (по умолчанию 8007)"),
        ],
    ),
    _Block(
        yaml_key="planning",
        title="MoveIt планирование",
        fields=[
            _Field("planning_group", "Группа планирования:", "iiwa_arm",
                   note="Группа планирования MoveIt из SRDF"),
            _Field("default_frame", "Система отсчёта по умолчанию:", "base_link"),
            _Field("default_planner", "Планировщик по умолчанию:", "ompl",
                   options=["ompl", "pilz_industrial_motion_planner", "chomp"]),
            _Field("planning_attempts", "Попыток планирования:", "3"),
        ],
    ),
    _Block(
        yaml_key="digital_twin",
        title="Цифровой двойник (Webots / RViz)",
        fields=[
            _Field("webots.transform", "Трансформ робота в сцене Webots (x y z, метры):", "-0.25 0 0.79"),
            _Field("webots.rotation", "Поворот робота в сцене Webots (ax ay az угол):", "0 0 1 0"),
            _Field("webots.controller_timer", "Шаг таймера контроллера Webots (мс):", "50"),
        ],
    ),
    _Block(
        yaml_key="robot",
        title="Подключение робота",
        fields=[
            _Field("name", "Имя модели робота:", "iiwa7"),
            _Field("ip", "IP-адрес робота:", "192.170.10.2",
                   note="IP контроллера KUKA на сетевом интерфейсе FRI"),
            _Field("port", "Порт FRI:", "30200"),
            _Field("fri_cycle_ms", "Цикл FRI (мс):", "10",
                   note="5 мс = 200 Гц, 10 мс = 100 Гц",
                   options=["10", "5"]),
            _Field("active_controller", "Активный ROS-контроллер:", "jtc",
                   note="jtc = JointTrajectoryController (MoveIt), forward = ForwardCommandController",
                   options=["jtc", "forward"]),
            _Field("joint_position_tau", "EMA tau фильтра положения (с):", "0.04",
                   note="Сглаживает команды положения перед отправкой в FRI"),
            _Field("joint_velocity_tau", "EMA tau фильтра скорости (с):", "0.01",
                   note="Убирает выбросы из оценки скорости конечной разностью"),
        ],
    ),
]


def _coerce(value: str, original: Any) -> Any:
    """Convert a string value to match the type of the original YAML value.
    Преобразует строковое значение к типу исходного значения YAML.
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


def _get_nested(mapping: Any, path: str) -> Any:
    """Return the value at a dot-separated path inside a nested YAML mapping, or None.
    Возвращает значение по пути с точками внутри вложенного YAML-словаря, или None.
    """
    keys = path.split(".")
    cur = mapping
    for k in keys:
        if cur is None or k not in cur:
            return None
        cur = cur[k]
    return cur


def _infer(value: str) -> Any:
    """Infer a bool / int / float / str from a raw string when there is no original
    value to match the type against (i.e. the key is new in the config).
    Выводит bool / int / float / str из строки, когда нет исходного значения для
    сопоставления типа (т.е. ключ новый в конфиге).
    """
    low = value.strip().lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        pass
    return value


def _set_nested(mapping: Any, path: str, value: Any) -> None:
    """Set the value at a dot-separated path, creating missing intermediate maps.

    If the leaf key already exists its type is preserved via _coerce; otherwise the
    type is inferred from the string with _infer. This makes the wizard tolerant of
    configs that do not yet contain every field (e.g. an older cobot-setting.yaml).

    Устанавливает значение по пути с точками, создавая отсутствующие промежуточные
    словари. Если конечный ключ уже есть — тип сохраняется через _coerce; иначе тип
    выводится из строки через _infer. Это делает мастер устойчивым к конфигам, где
    ещё нет всех полей (например, более старый cobot-setting.yaml).
    """
    keys = path.split(".")
    cur = mapping
    for k in keys[:-1]:
        if k not in cur or cur[k] is None:
            cur[k] = {}
        cur = cur[k]
    leaf = keys[-1]
    if leaf in cur and cur[leaf] is not None:
        cur[leaf] = _coerce(value, cur[leaf])
    else:
        cur[leaf] = _infer(value)


def _load_config() -> Any:
    """Load cobot-setting.yaml preserving comments and key order.
    Загружает cobot-setting.yaml, сохраняя комментарии и порядок ключей.
    """
    with open(_CONFIG_PATH, "r", encoding="utf-8") as fh:
        return _yaml.load(fh)


def _save_config(data: Any) -> None:
    """Write the modified YAML back to cobot-setting.yaml, preserving comments.
    Записывает изменённый YAML обратно в cobot-setting.yaml, сохраняя комментарии.
    """
    with open(_CONFIG_PATH, "w", encoding="utf-8") as fh:
        _yaml.dump(data, fh)


def _run_wizard(data: Any, blocks: List[_Block]) -> bool:
    """Walk every block: ask "Configure X?", and if yes step through its fields.
    Returns True if the user completed the wizard (config was saved), False on cancel.
    Проходит по каждому блоку: спрашивает "Настроить X?", и если да — проходит по полям.
    Возвращает True если мастер завершён (конфиг сохранён), False при отмене.
    """
    total = len(blocks)
    for bi, block in enumerate(blocks):
        ui.header(f"Блок {bi + 1}/{total}", block.title)
        if not ui.confirm(f"Настроить {block.title}?", default=True):
            continue
        for fi, f in enumerate(block.fields):
            yaml_val = _get_nested(data[block.yaml_key], f.key)
            current = str(yaml_val) if yaml_val is not None else str(f.default)
            step_note = f"Поле {fi + 1}/{len(block.fields)}"
            note = f"{step_note}\n{f.note}" if f.note else step_note
            if f.options:
                default_opt = current if current in f.options else f.options[0]
                value = ui.select(f.question, f.options, default_opt, note=note)
            else:
                value = ui.text(f.question, current, note=note)
            if value is None:
                ui.info("[yellow]Отменено.[/yellow]")
                return False
            _set_nested(data[block.yaml_key], f.key, value)

    _save_config(data)
    ui.done(True, f"Конфигурация сохранена в {_CONFIG_PATH.name}")
    return True


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("robot-setup", help="Configure cobot-setting.yaml interactively")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    ui.header("Настройка робота", "cobot-setting.yaml")

    if not _CONFIG_PATH.exists():
        ui.error(f"Конфиг не найден: {_CONFIG_PATH}")
        sys.exit(1)

    data = _load_config()

    # Убеждаемся что секция tool существует в данных (для старых конфигов)
    if "tool" not in data:
        data["tool"] = {"active": "patron"}

    tool_block = _build_tool_block()
    blocks = ([tool_block] if tool_block else []) + list(_BLOCKS)

    if not _run_wizard(data, blocks):
        return

    # Применяем выбранный инструмент: перезаписываем tool_active.xacro и iiwa7.srdf
    active_tool = str(data["tool"].get("active", "patron"))
    if not _TOOLS_YAML.exists():
        ui.info(f"[yellow]tools.yaml не найден ({_TOOLS_YAML}) — пропуск применения инструмента[/yellow]")
        return

    try:
        registry = _load_tools_registry()
        if active_tool not in registry:
            raise ValueError(f"Неизвестный инструмент '{active_tool}'. Доступны: {', '.join(registry)}")
        tool_cfg = dict(registry[active_tool])

        sys.path.insert(0, str(_PROJECT_DIR / "src" / "iiwa_utils"))
        from iiwa_utils.tool_manager import apply_tool
        apply_tool(tool_cfg=tool_cfg, xacro_out_path=_TOOL_ACTIVE_XACRO, srdf_path=_SRDF_PATH)

        # Синхронизируем planning.pose_link с tcp_link выбранного инструмента
        tcp_link = tool_cfg.get("tcp_link", "link_ee")
        if "planning" in data:
            data["planning"]["pose_link"] = tcp_link
            _save_config(data)

        ui.info(
            f"[green]✓[/green] Инструмент [bold]{active_tool}[/bold] применён: "
            f"tool_active.xacro, iiwa7.srdf обновлены, "
            f"planning.pose_link → [bold]{tcp_link}[/bold]."
        )
    except Exception as exc:
        ui.error(f"Не удалось применить инструмент: {exc}")
