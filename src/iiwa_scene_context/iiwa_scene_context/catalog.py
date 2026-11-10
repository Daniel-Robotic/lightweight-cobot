"""Server-side skill and object catalog for the Grant planner scene.

The catalog is the single source of truth for the ``scene_json`` contract.
It is deliberately pure Python: the snapshot builder, the graph validator and
the tests all import the same table.  The model never supplies its own
``skill_contracts``; only this module does.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from iiwa_experement.scenario import Tool, ScenarioSpec


SCENE_SCHEMA_VERSION = 1
CONTRACT_VERSION = 1
CATALOG_VERSION = 1

NATIVE_SKILLS = (
    'detect', 'approach', 'lower', 'grab', 'lift', 'transfer', 'align',
    'release', 'retreat', 'capture', 'verify', 'stop', 'wait_scene_clear',
    'inspect_state', 'notify_operator', 'dispense', 'dock_tool',
    'handoff_screw',
)
ALIAS_SKILLS = ('pick', 'move_to_storage', 'inspect')
AVAILABLE_SKILLS = NATIVE_SKILLS + ALIAS_SKILLS

TOOL_LABELS = {
    'vacuum': 'вакуумная присоска',
    'magnetic_tool': 'магнитный инструмент',
    'sealant_tool': 'инструмент нанесения герметика',
    'torque_tool': 'моментный инструмент',
}
TOOL_KINDS = {
    'vacuum': 'suction_cup',
    'magnetic_tool': 'magnet',
    'sealant_tool': 'dispenser',
    'torque_tool': 'torque_wrench',
}
_TOOL_SOCKET_SUFFIX = '_socket'
_ALLOWED_HOLE_IDS = ('hole_1', 'hole_2', 'hole_3', 'hole_4')
_ALLOWED_SEALANT_SIDES = ('side_1', 'side_2', 'side_3', 'side_4')


class CatalogError(ValueError):
    """The requested id does not exist in this scene catalog."""


# ``outcomes`` drives the validator: every result of a node must select
# exactly one edge, so every event listed here must be present.
SKILL_OUTCOMES: dict[str, dict[str, str]] = {
    'detect': {
        'found': 'Объект найден, актуальная локализация получена.',
        'not_found': 'Объект не найден, движение не выполнялось.',
        'failed': 'Наблюдение недоступно.',
    },
    'approach': {
        'success': 'Движение в безопасную точку над целью выполнено.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка движения.',
    },
    'lower': {
        'success': 'Осевое опускание к контакту выполнено.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка движения.',
    },
    'grab': {
        'success': 'Захват или установка инструмента подтверждены.',
        'grasp_failed': 'Захват не подтверждён; состояние нужно проверить.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка захвата.',
    },
    'lift': {
        'success': 'Осевой подъём выполнен.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка движения.',
    },
    'transfer': {
        'success': 'Перенос к цели на безопасной высоте выполнен.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка движения.',
    },
    'align': {
        'success': 'Ориентация совмещена на безопасной высоте.',
        'failed': 'Совмещение не подтверждено.',
    },
    'release': {
        'success': 'Объект отпущен в разрешённом месте.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Отпускание не подтверждено.',
    },
    'retreat': {
        'success': 'Осевой отвод выполнен.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка движения.',
    },
    'capture': {
        'success': 'Свежий кадр получен.',
        'failed': 'Кадр не получен.',
    },
    'verify': {
        'ok': 'Результат в допуске.',
        'not_ok': 'Результат вне допуска.',
        'failed': 'Невозможно измерить результат.',
    },
    'stop': {
        'success': 'Остановка подтверждена, состояние захвата сохранено.',
        'failed': 'Остановку подтвердить не удалось.',
    },
    'wait_scene_clear': {
        'clear': 'Зона свободна, продолжение разрешено.',
        'timeout': 'Условие не наступило за отведённое время.',
        'failed': 'Невозможно подтвердить условие.',
    },
    'inspect_state': {
        'empty': 'Захват пуст, деталь доступна для новой попытки.',
        'holding': 'Нужная деталь удерживается.',
        'placed': 'Деталь на месте, захват пуст.',
        'unknown': 'Состояние владения не подтверждено.',
    },
    'notify_operator': {
        'success': 'Уведомление оператору отправлено.',
        'failed': 'Доставка уведомления не подтверждена.',
    },
    'dispense': {
        'success': 'Участок герметика пройден.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка нанесения.',
    },
    'dock_tool': {
        'success': 'Инструмент возвращён в своё гнездо.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Стыковка не подтверждена.',
    },
    'handoff_screw': {
        'success': 'Винт передан виртуальному оператору у отверстия.',
        'grasp_failed': 'Передача не подтверждена; состояние нужно проверить.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка передачи.',
    },
    # Compatible aliases for graphs produced from the published datasets.
    'pick': {
        'success': 'Захват подтверждён.',
        'grasp_failed': 'Захват не подтверждён; состояние нужно проверить.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка захвата.',
    },
    'move_to_storage': {
        'success': 'Деталь установлена на опору, захват сохранён.',
        'human_in_scene': 'Шаг прерван из-за человека в зоне движения.',
        'collision': 'Исполнитель сообщил о столкновении.',
        'failed': 'Иная ошибка переноса.',
    },
    'inspect': {
        'ok': 'Результат в допуске.',
        'not_ok': 'Результат вне допуска.',
        'failed': 'Невозможно измерить результат.',
    },
}

SKILL_DESCRIPTIONS = {
    'detect': 'Найти объект, инструмент или место назначения; поза берётся из supervisor.',
    'approach': 'Переместить рабочий кончик в безопасную точку над целью и сохранить высоту.',
    'lower': 'Опустить рабочий кончик по оси к контакту.',
    'grab': 'Присоединить инструмент из гнезда или захватить деталь после контакта.',
    'lift': 'Поднять к сохранённой безопасной высоте без повторного захвата.',
    'transfer': 'Перенести удерживаемый объект к цели на безопасной высоте.',
    'align': 'Совместить ориентацию только на безопасной высоте.',
    'release': 'Отпустить деталь или вернуть инструмент строго в его гнездо.',
    'retreat': 'Сначала осевой отвод, затем уход в нейтральную позу.',
    'capture': 'Получить свежий кадр указанной камеры.',
    'verify': 'Проверить измеренное положение или доставку.',
    'stop': 'Остановить движение, сохранив состояние захвата.',
    'wait_scene_clear': 'Ограниченно ждать разрешения продолжить.',
    'inspect_state': 'Определить пустой захват, удержание, установку или неизвестность.',
    'notify_operator': 'Уведомить оператора и не двигать робота.',
    'dispense': 'Пройти вдоль заданного участка контура с инструментом герметика.',
    'dock_tool': 'Стыковать удерживаемый инструмент в его именованном гнезде.',
    'handoff_screw': 'Поднести винт к отверстию и передать виртуальному оператору.',
    'pick': 'Алиас grab для совместимых графов.',
    'move_to_storage': 'Алиас transfer+lower к опоре для совместимых графов.',
    'inspect': 'Алиас verify для совместимых графов.',
}

# Optional node payload keys, validated by the graph schema.  Fractions only
# apply to ``dispense``.
ALLOWED_PARAMS = {
    'dispense': ('start_fraction', 'end_fraction'),
}


@dataclass(frozen=True)
class SkillContract:
    name: str
    description: str
    outcomes: dict[str, str]

    def as_dict(self) -> dict[str, Any]:
        return {
            'description': self.description,
            'outcomes': dict(self.outcomes),
        }


@dataclass(frozen=True)
class SkillCatalog:
    """Object and skill registry for one scenario profile."""

    contract_version: int = CONTRACT_VERSION
    catalog_version: int = CATALOG_VERSION
    profile: str = 'grant'

    def skill_names(self) -> tuple[str, ...]:
        return AVAILABLE_SKILLS

    def skill_contracts(self) -> dict[str, dict[str, Any]]:
        return {
            name: SkillContract(
                name, SKILL_DESCRIPTIONS[name], SKILL_OUTCOMES[name],
            ).as_dict()
            for name in AVAILABLE_SKILLS
        }

    def outcomes(self, skill: str) -> dict[str, str]:
        if skill not in SKILL_OUTCOMES:
            raise CatalogError(f'unknown skill {skill!r}')
        return dict(SKILL_OUTCOMES[skill])

    def tool_socket_id(self, tool_name: str) -> str:
        return f'{tool_name}{_TOOL_SOCKET_SUFFIX}'

    def component_ids(self, spec: ScenarioSpec) -> tuple[str, ...]:
        ids = ['fixture', 'body']
        if spec.cap_state != 'absent':
            ids.append('cap')
        if spec.tray_pose is not None:
            ids.append('tray')
            ids.extend(screw.name for screw in spec.screws)
        return tuple(ids)

    def tool_ids(self, spec: ScenarioSpec) -> tuple[str, ...]:
        return tuple(tool.name for tool in spec.tools)

    def destination_ids(self, spec: ScenarioSpec) -> tuple[str, ...]:
        ids = ['work', 'hatch_seat', *_ALLOWED_HOLE_IDS]
        ids.extend(self.tool_socket_id(tool.name) for tool in spec.tools)
        ids.extend(spec.sealant_sides)
        return tuple(ids)

    def object_ids(self, spec: ScenarioSpec) -> tuple[str, ...]:
        """Every id that may appear as ``object`` in a graph."""
        return self.component_ids(spec) + self.tool_ids(spec)

    def target_ids(self, spec: ScenarioSpec) -> tuple[str, ...]:
        """Every id that may appear as ``target`` in a graph."""
        ids = list(self.destination_ids(spec))
        # The object itself is a legal target for pickup/detection nodes and
        # for explicit socket placement (dock_tool uses the socket id).
        ids.extend(self.component_ids(spec))
        ids.extend(self.tool_ids(spec))
        return tuple(ids)

    def component_kind(self, object_id: str, spec: ScenarioSpec) -> str:
        if object_id == 'fixture':
            return 'fixture'
        if object_id == 'body':
            return 'body'
        if object_id == 'cap':
            if spec.cap_state == 'absent':
                raise CatalogError('hatch is absent in this scene')
            return 'hatch'
        if object_id == 'tray':
            if spec.tray_pose is None:
                raise CatalogError('tray is absent in this scene')
            return 'tray'
        if any(screw.name == object_id for screw in spec.screws):
            return 'screw'
        if object_id in self.tool_ids(spec):
            return 'tool'
        raise CatalogError(f'unknown component {object_id!r}')

    def destination_kind(self, destination_id: str, spec: ScenarioSpec) -> str:
        if destination_id == 'work':
            return 'workspace'
        if destination_id == 'hatch_seat':
            return 'seat'
        if destination_id in _ALLOWED_HOLE_IDS:
            return 'hole'
        if destination_id.endswith(_TOOL_SOCKET_SUFFIX):
            tool = destination_id[: -len(_TOOL_SOCKET_SUFFIX)]
            if tool in self.tool_ids(spec):
                return 'tool_socket'
        if destination_id in spec.sealant_sides:
            return 'contour'
        raise CatalogError(f'unknown destination {destination_id!r}')

    def tool_socket_owner(self, socket_id: str) -> str | None:
        if socket_id.endswith(_TOOL_SOCKET_SUFFIX):
            return socket_id[: -len(_TOOL_SOCKET_SUFFIX)]
        return None

    def validate_ids(self, spec: ScenarioSpec) -> None:
        objects = set(self.object_ids(spec))
        targets = set(self.target_ids(spec))
        if 'cap' in objects and 'hatch_seat' not in targets:
            raise CatalogError('a present hatch must have hatch_seat')
        for tool in spec.tools:
            socket_id = self.tool_socket_id(tool.name)
            if socket_id not in targets:
                raise CatalogError(f'tool {tool.name!r} has no named socket')
        for side in spec.sealant_sides:
            if side not in targets:
                raise CatalogError(f'sealant side {side!r} is not a destination')


DEFAULT_CATALOG = SkillCatalog()


def build_component_entry(
    catalog: SkillCatalog,
    spec: ScenarioSpec,
    object_id: str,
    state: dict[str, Any],
) -> dict[str, Any]:
    kind = catalog.component_kind(object_id, spec)
    entry: dict[str, Any] = {
        'id': object_id,
        'name': _component_label(kind, object_id),
        'attributes': {'kind': kind, 'movable': kind not in ('fixture', 'body')},
        'dependencies': [],
    }
    if kind == 'hatch':
        entry['destination'] = 'hatch_seat'
    elif kind == 'screw':
        entry['destination'] = ''
        entry['attributes']['allowed_destinations'] = list(_ALLOWED_HOLE_IDS)
    else:
        entry['destination'] = ''
    if object_id == 'cap':
        entry['state'] = str(state.get('cap', {}).get('state', 'unknown'))
    elif kind == 'screw':
        screw = state.get('screws', {}).get(object_id, {})
        entry['state'] = str(screw.get('state', 'unknown'))
        entry['hole'] = screw.get('hole')
    return entry


def _component_label(kind: str, object_id: str) -> str:
    labels = {
        'fixture': 'оснастка',
        'body': 'корпус',
        'hatch': 'люк',
        'tray': 'лоток с винтами',
        'screw': 'винт',
    }
    if kind == 'screw':
        return f'винт {object_id.split("_")[-1]}'
    return labels.get(kind, object_id)


def destination_entry(
    catalog: SkillCatalog,
    spec: ScenarioSpec,
    destination_id: str,
    state: dict[str, Any],
) -> dict[str, Any]:
    kind = catalog.destination_kind(destination_id, spec)
    entry: dict[str, Any] = {
        'id': destination_id,
        'name': _destination_label(kind, destination_id),
        'kind': kind,
        'capacity': 1,
    }
    if kind == 'hole':
        hole = state.get('holes', {}).get(destination_id, {})
        entry['state'] = str(hole.get('state', 'unknown'))
        entry['screw'] = hole.get('screw')
    elif kind == 'tool_socket':
        tool = catalog.tool_socket_owner(destination_id)
        tool_state = state.get('tools', {}).get(tool, {})
        entry['state'] = (
            'occupied'
            if tool_state.get('state') == 'docked'
            else 'free'
        )
        entry['tool'] = tool
    elif kind == 'seat':
        entry['state'] = 'empty'
    else:
        entry['state'] = 'available'
    return entry


def _destination_label(kind: str, destination_id: str) -> str:
    if destination_id == 'work':
        return 'рабочая позиция'
    if destination_id == 'hatch_seat':
        return 'посадочное место люка на корпусе'
    if kind == 'hole':
        return f'отверстие {destination_id.split("_")[-1]}'
    if kind == 'tool_socket':
        return f'гнездо {destination_id[: -len(_TOOL_SOCKET_SUFFIX)]}'
    if kind == 'contour':
        return f'участок герметика {destination_id}'
    return destination_id


def tool_entry(
    catalog: SkillCatalog,
    spec: ScenarioSpec,
    tool: Tool,
    state: dict[str, Any],
) -> dict[str, Any]:
    tool_state = state.get('tools', {}).get(tool.name, {})
    return {
        'id': tool.name,
        'name': TOOL_LABELS.get(tool.name, tool.name),
        'kind': TOOL_KINDS.get(tool.name, 'tool'),
        'socket': catalog.tool_socket_id(tool.name),
        'state': str(tool_state.get('state', 'unknown')),
    }
