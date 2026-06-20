# Конфигурация системы

## Главный конфигурационный файл

Все параметры системы хранятся в одном файле — **`cobot-setting.yaml`** в корне проекта.
Это единственный источник истины для IP-адреса робота, портов, путей к конфигам, настроек планировщика и веб-сервера.

!!! danger "Не редактируйте файл вручную"
    Используйте только команду `cobot robot-setup` — интерактивный мастер валидирует значения и не допускает синтаксических ошибок. Ручное редактирование YAML может привести к ошибкам парсинга и невозможности запуска системы.

```bash
cobot robot-setup
```

---

## Блок `robot` — параметры робота

Отвечает за соединение с физическим контроллером KUKA по протоколу FRI.

```yaml
robot:
  name: "iiwa7"
  ip: "192.170.10.2"
  port: 30200
  fri_cycle_ms: 10
  joint_position_tau: 0.04
  joint_velocity_tau: 0.01
  active_controller: "jtc"
  description: pkg://iiwa_description/urdf/iiwa7.urdf.xacro
```

| Параметр | Описание | Рекомендации |
|---|---|---|
| `name` | Модель робота | Не менять — `iiwa7` |
| `ip` | IP-адрес контроллера KUKA | **Изменить** на реальный IP вашего контроллера |
| `port` | UDP-порт FRI | По умолчанию `30200` — менять только при конфликте портов |
| `fri_cycle_ms` | Период цикла FRI: `5` мс = 200 Гц, `10` мс = 100 Гц | `10` для стабильной работы, `5` для высокоточных задач |
| `joint_position_tau` | EMA-фильтр позиций [с] — сглаживает команды перед отправкой | Уменьшать для более быстрой реакции, увеличивать при вибрациях |
| `joint_velocity_tau` | EMA-фильтр скорости [с] — убирает выбросы конечных разностей | Аналогично `joint_position_tau` |
| `active_controller` | Режим управления: `jtc` (MoveIt / JointTrajectory) или `forward` (прямое управление) | `jtc` для большинства задач |
| `description` | Путь к URDF-описанию робота | Не менять |

---

## Блок `digital_twin` — симулятор

Настройки виртуальной среды Webots и визуализации в RViz.

```yaml
digital_twin:
  webots:
    world: pkg://iiwa_description/worlds/iiwa.wbt
    transform: "-0.25 0 0.79"
    rotation: "0 0 1 0"
    controller_timer: "50"
    cameras:
      - pkg://iiwa_config/config/cameras/d455_top.yaml
  rviz:
    config: pkg://iiwa_config/config/rviz/rviz_moveit.rviz
```

| Параметр | Описание |
|---|---|
| `webots.world` | Путь к `.wbt`-миру симулятора |
| `webots.transform` | Смещение базы робота в мире `[x y z]` в метрах |
| `webots.rotation` | Ориентация базы `[x y z угол]` в радианах |
| `webots.cameras` | Список YAML-конфигов подключённых камер |
| `rviz.config` | Путь к конфигурации RViz |

---

## Блок `tool` — активный инструмент

Указывает, какой захват или инструмент закреплён на роботе.

```yaml
tool:
  active: "patron"
```

| Значение | Описание |
|---|---|
| `none` | Без инструмента |
| `patron` | Захват «Patron» (патрон) |

Список доступных инструментов находится в файле `src/iiwa_config/config/tools.yaml`. Для добавления нового инструмента необходимо описать его там, после чего указать имя в `tool.active`.

---

## Блок `planning` — планирование движений

Параметры MoveIt 2 и конфигурации траекторного планировщика.

```yaml
planning:
  pose_link: "tcp"
  planning_group: "iiwa_arm"
  default_frame: "base_link"
  default_planner: "ompl"
  planning_attempts: 3
```

| Параметр | Описание | Рекомендации |
|---|---|---|
| `pose_link` | TCP-линк для декартовых целей | Соответствует фрейму в URDF — не менять без изменения URDF |
| `planning_group` | Группа планирования из SRDF | Не менять — `iiwa_arm` |
| `default_frame` | Система отсчёта по умолчанию | Не менять — `base_link` |
| `default_planner` | Планировщик: `ompl`, `pilz_industrial_motion_planner` | `ompl` — универсальный; `pilz` — для предсказуемых траекторий |
| `planning_attempts` | Число попыток планирования при неудаче | Увеличивать при сложных траекториях |

---

## Блок `web` — REST API и MCP-сервер

FastAPI-сервер для управления роботом через HTTP и MCP (интеграция с AI-агентами).

```yaml
web:
  enabled: true
  host: "0.0.0.0"
  port: 8007
  endpoints: pkg://iiwa_config/config/api_endpoints.yaml
  joint_limits: pkg://iiwa_config/config/moveit/joint_limits.yaml
```

| Параметр | Описание |
|---|---|
| `enabled` | Включить (`true`) или отключить (`false`) веб-сервер |
| `host` | Адрес прослушивания: `0.0.0.0` — все интерфейсы, `127.0.0.1` — только локально |
| `port` | Порт HTTP API (по умолчанию `8007`) |
| `endpoints` | Путь к описанию REST-эндпоинтов |
| `joint_limits` | Путь к файлу ограничений суставов для валидации команд |

После запуска REST API доступен по адресу `http://<host>:8007`, MCP — по пути `/mcp`.

---

## Блок `foxglove` — мониторинг через Foxglove Studio

[Foxglove Studio](https://foxglove.dev/) — инструмент визуализации и мониторинга ROS 2 топиков в реальном времени.

```yaml
foxglove:
  enabled: true
  port: 8765
  debug: false
  address: 0.0.0.0
```

| Параметр | Описание |
|---|---|
| `enabled` | Включить или отключить Foxglove Bridge |
| `port` | WebSocket-порт для подключения Foxglove Studio (по умолчанию `8765`) |
| `debug` | Подробное логирование bridge-процесса |
| `address` | Адрес прослушивания WebSocket |

Остальные параметры блока (`tls`, `topic_whitelist`, `min_qos_depth` и др.) относятся к продвинутой конфигурации и в большинстве случаев менять их не требуется.

---

## Что менять, что оставить

| | Параметр | Действие |
|---|---|---|
| ✅ | `robot.ip` | **Обязательно изменить** на IP вашего контроллера |
| ✅ | `robot.fri_cycle_ms` | Выбрать `10` (стандарт) или `5` (высокая частота) |
| ✅ | `tool.active` | Указать активный инструмент |
| ✅ | `web.enabled` | Выключить (`false`), если веб-интерфейс не нужен |
| ⚠️ | `robot.active_controller` | Менять только при намеренном переключении режима управления |
| ⚠️ | `planning.*` | Менять при необходимости другого планировщика или параметров |
| ❌ | `robot.description` | Не трогать — путь к URDF |
| ❌ | `controller.moveit.*` | Не трогать — пути к конфигам MoveIt внутри пакетов |
| ❌ | `digital_twin.webots.world` | Не трогать без знания структуры Webots-миров |
