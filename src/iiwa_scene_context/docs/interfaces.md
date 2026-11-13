# Интерфейсы `iiwa_scene_context`

Версия схемы сцены: **2**. Версия контракта: **2**. Версия каталога: **2**.
Весь JSON — UTF-8, идентификаторы объектов стабильны и совпадают с Webots DEF.

## ROS-интерфейсы

| Направление | Имя | Тип | Примечание |
| --- | --- | --- | --- |
| сервис | `/iiwa_scene_context/get_scene` | `std_srvs/Trigger` | `success=true`, `message` = JSON конверта; иначе причина (сцена не готова / устарела / нет состояния). |
| топик | `/iiwa_scene_context/scene_json` | `std_msgs/String` | Тот же конверт. QoS: reliable, transient-local, depth 1; поздний подписчик получает последний снимок. |
| вход | `/iiwa_experement/object_state` | `std_msgs/String` | Живое состояние supervisor: владение, состояния и позы объектов. |
| вход | `/iiwa_experement/scene_ready` | `std_srvs/Trigger` | Опрашивается узлом. До `success=true` снимок не публикуется. |

Узел отвечает `success=false`, пока сцена не готова, пока не пришло первое
состояние supervisor или пока последнее состояние старше `freshness_sec`
(по умолчанию 2.0 с). Устаревший снимок нельзя трактовать как актуальный.

## Конверт снимка

```json
{
  "schema_version": 2,
  "contract_version": 2,
  "catalog_version": 2,
  "experiment_id": 20,
  "state_version": 7,
  "timestamp": 42.0,
  "scene_json": { "...": "см. ниже" }
}
```

* `state_version` — дискретная версия supervisor. Она растёт при подтверждённом
  изменении владения; обновление позы удерживаемого предмета её не меняет.
* `scene_revision` **не входит** в публичный контракт: клиент не передаёт и не
  получает его. Исполнитель сам хранит внутренний счётчик для защиты от
  внешнего изменения сцены во время выполнения.
* `timestamp` — время получения снимка. Потребитель сам проверяет свежесть.

## `scene_json`

Поля: `schema_version`, `contract_version`, `catalog_version`, `profile`,
`experiment_id`, `domain`, `pose_conventions`, `components`, `destinations`,
`tools`, `initial_state`, `cameras`, `available_skills`, `skill_contracts`,
`action_rules`, `service_objects`, `recovery_attempts_per_component`.

`components` содержит только реально созданные объекты выбранного
эксперимента: всегда `fixture`, `body`; условно `cap`, `tray`, `screw_1`…
`screw_10`. `tools` содержит только присутствующие инструменты.
`destinations` содержит `work`, `hatch_seat`, `hole_1`…`hole_4`, гнездо
`<tool>_socket` для каждого инструмента и только те участки герметика
`side_i`, которые заданы в сцене. Для каждого участка публикуются начальная и
конечная позы.

### Позы

Единые правила: `frame_id="world"`, метры, кватернион `xyzw`.

```json
{
  "frame_id": "world",
  "position": {"x": 0.3, "y": 0.12, "z": 0.823},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.3826834, "w": 0.9238795},
  "source": "webots_supervisor",
  "stamp": 42.0
}
```

* `source="webots_supervisor"` — живая поза из supervisor. Пока предмет
  удерживается, публикуется его текущая поза, а не начальная.
* `source="scenario_manifest"` — запасная начальная поза, если supervisor
  ещё не прислал объект (например, в unit-тестах).
* `source="executor_geometry"` — производная цель. Она вычисляется теми же
  функциями `world_target`, `sealant_point` и `tcp_goal`, что использует
  исполнитель, поэтому поза в `scene_json` и цель движения не расходятся.

Позы есть у `components` и `tools` в поле `pose`. У мест назначения:
`pose` (поза объекта/рабочей точки) и `tcp_pose` (цель TCP). Участок
герметика содержит `start`, `end`, `start_tcp`, `end_tcp`. Числа проверяются
на конечность, кадр — на `world`, кватернион — на нормировку.

### Правила действий

`action_rules` — компактные машиночитаемые правила для каждого доступного
навыка:

```json
"grab": {
  "object_types": ["hatch", "screw", "tool"],
  "target_types": [],
  "objects": ["cap", "screw_1", "vacuum", "..."],
  "targets": [],
  "required_tools": [],
  "required_tool_by_kind": {"hatch": "vacuum", "screw": "magnetic_tool"},
  "preconditions": ["gripper is empty"],
  "result": "tool mounted or part attached to the mounted tool"
}
```

Правила строятся из выбранной сцены, поэтому отсутствующие ID и навыки не
рекламируются. В частности:

* `grab(cap)` требует `vacuum`, `grab(screw_i)` — `magnetic_tool`;
* `dispense` доступен только при наличии `sealant_tool` и участка `side_i`;
* `dock_tool` требует предшествующий `transfer(<tool>, <tool>_socket)` в
  собственное гнездо;
* `handoff_screw` только передаёт винт виртуальному оператору у отверстия;
  вставки и затяжки нет, отверстие остаётся пустым;
* `tighten` не рекламируется, пока навык не реализован;
* `move_to_storage` не применяется к инструменту и не ведёт `sealant_tool` в
  `hatch_seat`.

Сгенерированный пример для эксперимента 20:
`docs/scene_json_experiment_20.json`. Он создаётся тем же `build_scene`, а не
заменяет динамическую генерацию; тест сверяет пример с генератором.
