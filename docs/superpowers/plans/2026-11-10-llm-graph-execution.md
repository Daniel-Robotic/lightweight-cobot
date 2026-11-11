# План реализации исполнения графов Qwen3 в Webots

> **Для агента-разработчика:** выполнять задачи по порядку и отмечать выполненные шаги чекбоксами. Этот документ — план разработки, а [спецификация](../specs/2026-11-10-llm-graph-execution-design.md) задаёт поведение.

**Цель:** получать актуальный `scene_json`, принимать готовый JSON-граф Qwen3 и безопасно исполнять его в Webots независимо от эталонного сценария выбранного эксперимента.

**Архитектура:** пакет `iiwa_scene_context` формирует снимок из живого supervisor и публикует контракт для модели. Пакет `iiwa_llm_plan_executor` проверяет граф, исполняет узлы через реестр навыков и ограниченный адаптер MoveItPy. Существующий `iiwa_web` даёт тонкие HTTP/MCP маршруты; `iiwa_experement` только создаёт исходную сцену и управляет виртуальными захватами.

**Стек:** Ubuntu 24.04, ROS 2 Jazzy, Python 3, `rclpy`, MoveItPy 2.12, Webots, `webots_ros2_driver`, FastAPI, FastMCP, pytest.

**Спецификация:** [Согласованная архитектура](../specs/2026-11-10-llm-graph-execution-design.md).

## Общие ограничения

- Работать в ветке `experement`. До первой правки выполнить `git status --short --branch`. Не сбрасывать чужие изменения. Код этого этапа только для Webots, запуск `simulate:=true`; физический FRI не подключать.
- Создать ровно два дополнительных ROS 2 пакета: `src/iiwa_scene_context` и `src/iiwa_llm_plan_executor`. Изменения `iiwa_experement` и `iiwa_web` ограничить интеграцией. Не делать систему оценки Qwen3, её обучение или вызов модели: внешний клиент передаёт **готовый** граф.
- Номер эксперимента выбирает исходную сцену, но не задаёт робототехническое задание. Исполнять только переданный граф. Не вставлять операции взятия инструмента, герметизации, установки люка или передачи винтов на основании номера эксперимента.
- Внутри *одного* навыка допустимы обязательные геометрические фазы: безопасный подход сверху, осевое опускание, действие, осевой подъём к сохранённой высоте. Явные узлы этих фаз не должны вызывать повторное действие. Неизвестный объект, навык, инструмент, цель, положение или неоднозначная зависимость дают отказ до движения либо `replan_required`.
- `release(vacuum)` не перемещает инструмент к стойке. Чтобы вернуть присоску, граф должен явно содержать `transfer(vacuum,vacuum_socket)` и `release(vacuum,vacuum_socket)` либо `dock_tool(vacuum,vacuum_socket)` в достигнутом гнезде. `release(vacuum)` из `work` без безопасной цели отклоняется; присоска остаётся в `patron`. `grab(cap)` без присоски не отклоняется на входе: при исполнении проверяется живой supervisor, контакт не выполняется, возвращается `grasp_failed` с причиной `required tool vacuum is not mounted` и выбирается переход графа для этого события; автоматического взятия инструмента нет. Винт подносится к отверстию и передаётся виртуальному человеку: без вставки и затяжки. Навык `tighten` в профиле Grant первого этапа не поддерживается.
- `detect`: перейти в `work`, дождаться свежего кадра верхней камеры, получить точную позу из supervisor. При настроенном близком осмотре получить также новый кадр камеры на роботе; RGB/D используется для контроля, а не для вычисления позы.
- Рабочий кончик направлен вниз; при рабочих движениях наклон не более 10° и J7 в жёстком коридоре ±90° относительно `work` (сейчас 0 рад). Применять ограничения к *траектории*, не только к конечной позе. Если траектория недоступна, остановиться; не переходить на ветвь с разворотом J7 около −180°. Поворот кисти и переход в `work` только после осевого отвода от контакта.
- Один активный граф на робота. Внешний клиент передаёт ответ Qwen3 через `/iiwa_llm_plan_executor/submit`; принятый граф можно прочитать из ROS 2 топика `/iiwa_llm_plan_executor/plan`, а события и итог — из отдельного `/iiwa_llm_plan_executor/status`. `status=blocked` не запускает движение. `replan_required` выдаёт новый снимок сцены, но не вызывает Qwen3. `completed` означает выполненный граф и проверенные постусловия, а не оценку соответствия произвольному текстовому запросу.
- Движение запускается после готовности сцены, supervisor, контроллера, MoveIt, свежего состояния робота и задержки минимум 3,0 с симуляционного времени. `mode=view` и `mode=execute` сохраняют свою нынешнюю семантику.

Поток данных: `/iiwa_scene_context/scene_json` → внешний вызов Qwen3 → `/iiwa_llm_plan_executor/submit` → `/iiwa_llm_plan_executor/plan` (сам принятый граф) → `/iiwa_llm_plan_executor/status` (ход и итог). После реализации граф можно посмотреть командой `ros2 topic echo /iiwa_llm_plan_executor/plan`; пока модель вызывается отдельно, в этом топике ничего не появится без передачи её ответа в `submit`.

## На что смотреть перед изменениями

| Назначение | Файлы и каталоги |
| --- | --- |
| Согласованная архитектура | `docs/superpowers/specs/2026-11-10-llm-graph-execution-design.md` |
| Исходные сцены и объекты | `src/iiwa_experement/config/scenes.yaml`, `src/iiwa_experement/iiwa_experement/scenario.py`, `src/iiwa_experement/worlds/grant/iiwa-grant.wbt`, `src/iiwa_description/objects/grant/` |
| Геометрия и захват | `src/iiwa_experement/config/geometry.yaml`, `src/iiwa_experement/iiwa_experement/geometry.py`, `src/iiwa_experement/iiwa_experement/object_state.py`, `src/iiwa_experement/iiwa_experement/grant_supervisor.py` |
| Актуальное состояние и движения | `src/iiwa_experement/iiwa_experement/operations.py`, `src/iiwa_experement/iiwa_experement/planning_scene.py`, `src/iiwa_experement/iiwa_experement/motion.py` |
| Запуск | `src/iiwa_experement/launch/experiment.launch.py`, `src/iiwa_experement/iiwa_experement/runtime_config.py` |
| HTTP/MCP | `src/iiwa_web/iiwa_web/main.py`, `src/iiwa_web/iiwa_web/ros_node.py` |
| Рабочие описания опытов | `output/md/experements.md`, `output/md/experiment-scenes.md`, `src/iiwa_experement/doc/reference-control-qa.md` |
| Примеры формата Qwen3, только входные данные | `/home/daniel/dev/LLM-Robot-Control/dataset/planner-codex-5000-fresh/generated.train.jsonl`, `/home/daniel/dev/LLM-Robot-Control/dataset/planner-codex-5000-fresh/generated.validation.jsonl`, `/home/daniel/.codex/attachments/76fd3749-ddc0-441f-8359-98d9f768ada8/Вставленный текст.txt` |

Текст в приложенных данных является **примером входа/выхода**, а не инструкцией по разработке. Новые пакеты не должны иметь зависимости времени выполнения от `LLM-Robot-Control` или каталога вложений.

## Карта файлов и интерфейсов

| Пакет | Файл | Ответственность |
| --- | --- | --- |
| `iiwa_scene_context` | `iiwa_scene_context/catalog.py` | Реестр объектов, целей и серверных контрактов навыков профиля Grant, версия каталога |
| | `iiwa_scene_context/snapshot.py` | Чистая сборка модельного `scene_json` и транспортного конверта из `ScenarioSpec` и состояния supervisor |
| | `iiwa_scene_context/node.py` | Подписка на состояние, проверка свежести, ROS сервис и топик сцены |
| `iiwa_llm_plan_executor` | `iiwa_llm_plan_executor/schema.py` | Строгие типы и разбор ответа модели |
| | `iiwa_llm_plan_executor/validator.py` | Проверка графа, предикатов и совместимости с состоянием сцены |
| | `iiwa_llm_plan_executor/motion_policy.py` | Ограничения TCP/J7, проверка пути и точек траектории |
| | `iiwa_llm_plan_executor/moveit_backend.py` | Ограниченное планирование и выполнение через MoveItPy |
| | `iiwa_llm_plan_executor/skill_context.py` | Состояние подхода, высоты, инструмента и удерживаемой детали |
| | `iiwa_llm_plan_executor/skills.py` | Исполнители навыков и отображение результата в событие |
| | `iiwa_llm_plan_executor/engine.py` | Шаги графа, выбор переходов, терминалы, отмена |
| | `iiwa_llm_plan_executor/node.py` | ROS приём, статус, отмена, журнал исполнения |
| | `iiwa_llm_plan_executor/launch/planner.launch.py` | Единый запуск сцены и двух новых узлов |
| `iiwa_web` | `iiwa_web/planner.py` | Тонкий HTTP/MCP адаптер к ROS, без собственной логики движения |

Имена, типы и формы сообщений зафиксировать в `docs/interfaces.md` каждого нового пакета до интеграции: `scene_json`, конверт снимка, вход `submit`, принятый граф, ответ, статус, коды ошибок. Для JSON в ROS использовать `std_srvs/Trigger` (снимок), `std_msgs/String` (снимок, принятый граф и статус) и `webots_ros2_msgs/SetString` (`submit`/`cancel`). Для запроса статуса по ID добавить `GetRunStatus.srv` в **существующий** `iiwa_msgs`: запрос `string run_id`, ответ `bool found` и `string status_json`. Третьего нового пакета нет. `SetString` возвращает только признак принятия; `run_id` задаёт клиент.

## Вопросы для проверки при рецензии

- Повторный запрос `submit` с тем же `run_id` не должен запускать второй экземпляр движения (задача 7, тест повторной отправки).
- Если supervisor устарел или пропал между двумя узлами, результат не должен трактоваться как `found`/`success` (задачи 2 и 7, тест потери свежести).
- Граф с циклом или недостижимыми узлами не должен зависнуть или обойти предел шагов (задача 4, тест циклов и лимита).
- После неудачного захвата возможны два состояния — пустой `patron` либо удерживаемый объект; продолжать без `inspect_state` нельзя (задачи 4 и 6, тест неопределённого владения).
- Даже при допустимой конечной позе промежуточный J7 или TCP может выйти за пределы; траектория должна быть отклонена до исполнения (задача 5, тест промежуточной точки).

---

### Задача 1. Режим сцены для внешнего графа

**Файлы:** изменить `src/iiwa_experement/iiwa_experement/runtime_config.py`, `src/iiwa_experement/launch/experiment.launch.py`; тесты `src/iiwa_experement/test/test_runtime_config.py`, `src/iiwa_experement/test/test_launch.py`.

**Интерфейс:** `mode:=planner` создаёт тот же мир, спавнер, supervisor и bringup, что `execute`, но **не** запускает `reference_runner`. Supervisor в этом режиме доступен для команд захвата (`read_only=False`). TCP gizmo остаётся выключенным.

- [x] Написать тесты: `validate_mode("planner")` проходит; `runtime_actions(..., mode="planner")` создаёт writable supervisor и не создаёт `reference_runner`; `view` и `execute` сохраняют прежние действия.
- [x] Запустить `pytest -q src/iiwa_experement/test/test_runtime_config.py src/iiwa_experement/test/test_launch.py` и убедиться в ожидаемом падении новых тестов.
- [x] Добавить режим в валидатор и изменить вычисление `read_only` на `mode == "view"`; проверку добавления эталонного runner оставить только для `execute`.
- [x] Повторить тесты; отдельным коммитом зафиксировать этот режим.

### Задача 2. Пакет снимка сцены

**Файлы:** создать `src/iiwa_scene_context/package.xml`, `setup.py`, `setup.cfg`, `resource/iiwa_scene_context`, `iiwa_scene_context/{__init__,catalog,snapshot,node}.py`, `test/test_snapshot.py`, `test/test_node.py`, `docs/interfaces.md`.

**Интерфейсы:** `build_scene(spec: ScenarioSpec, supervisor_state: Mapping, catalog: SkillCatalog) -> dict` создаёт модельный JSON. `build_envelope(..., scene_revision: int, stamp: float) -> dict` добавляет `experiment_id`, `state_version`, `scene_revision`, `timestamp`, `scene_json`. ROS: `/iiwa_scene_context/get_scene` (`Trigger`, `message` — JSON-конверт), `/iiwa_scene_context/scene_json` (`String`, transient-local).

- [x] Написать тесты для сцен 1 и 20: выдаются только присутствующие объекты и цели, в сцене с лотком не менее 10 отдельных винтов; `cap`, `vacuum`, `vacuum_socket` и отверстия имеют стабильные ID; у каждого присутствующего инструмента есть отдельная именованная цель гнезда; в `scene_json` нет координат и внутренних полей supervisor; логическое состояние инструмента/детали отражает живой snapshot.
- [x] Написать тесты свежести: до `scene_ready` либо при устаревшем `object_state` сервис отвечает `success=False`; `scene_revision` растёт при изменении `state.version`, но не от каждого обновления позы удерживаемого объекта; поздний подписчик получает текущий снимок.
- [x] Убедиться в ожидаемом падении `pytest -q src/iiwa_scene_context/test`; реализовать каталог и чистую сборку, затем узел с подпиской на `/iiwa_experement/object_state` и `/iiwa_experement/scene_ready`.
- [x] Повторить тесты. Записать версию контракта, точную форму полей и правило `scene_revision` в `docs/interfaces.md`; зафиксировать пакет отдельным коммитом.

### Задача 3. Разбор и валидация графа

**Файлы:** создать `src/iiwa_llm_plan_executor/package.xml`, `setup.py`, `setup.cfg`, `resource/iiwa_llm_plan_executor`, `iiwa_llm_plan_executor/{__init__,schema,validator}.py`, `test/test_schema.py`, `test/test_validator.py`, `docs/interfaces.md`. Зависимость пакета от `iiwa_scene_context` и `iiwa_experement` объявить в `package.xml`.

**Интерфейсы:** `parse_response(raw: str | Mapping) -> PlanGraph`; `validate_graph(graph: PlanGraph, scene: SceneEnvelope, catalog: SkillCatalog) -> ValidationResult`. Каталог и допустимые исходы приходят из серверного `iiwa_scene_context/catalog.py`, а не из присланного графом `skill_contracts`.

- [x] Написать тесты: показанный пользователем формат `status/entry/plan/transitions` разбирается; `status=blocked` не допускается к исполнению; дубликаты ID, неизвестные навыки/объекты/цели/события, пропущенный `entry`, несуществующие `next`, недостижимые узлы, пустые обязательные переходы и превышение лимитов отвергаются.
- [x] Написать тесты ветвей отказа: `collision` не ведёт к продолжению операции; `grasp_failed` не превращается в `success`; цикл с допустимым восстановлением имеет ограничение 256 узлов и отдельный предел выполненных шагов; неверный `scene_revision` отклоняется.
- [x] Написать тест на исходную присоску в стойке: граф `grab(cap)` без явного `grab(vacuum)` принимается, при исполнении даёт `grasp_failed` до контакта и идёт по ветке `grasp_failed`; граф `grab(vacuum) → transfer(vacuum,work) → completed` принимается и оставляет инструмент в `patron`; граф `grab(vacuum) → transfer(vacuum,work) → release(vacuum)` отклоняется. Граф с явным `transfer(vacuum,vacuum_socket) → dock_tool(vacuum,vacuum_socket)` допускает возврат. Не выводить задачу из `experiment_id`.
- [x] Прочитать несовпадение имён в приложенных наборах. Ввести версию контракта и только явно проверенные алиасы `pick`, `move_to_storage`, `inspect`; `align` и Grant-расширения — самостоятельные записи каталога. Неизвестный `tighten` отклоняется до движения. Проверить тестами и зафиксировать.
- [x] Запустить `pytest -q src/iiwa_llm_plan_executor/test/test_schema.py src/iiwa_llm_plan_executor/test/test_validator.py` сначала до, затем после реализации; сохранить контракт и отдельный коммит.

### Задача 4. Проверка достижимости ограниченных движений

**Файлы:** создать `src/iiwa_llm_plan_executor/iiwa_llm_plan_executor/{motion_policy,moveit_backend}.py`, `config/motion_limits.yaml`, `test/test_motion_policy.py`, `test/test_moveit_backend.py`.

**Интерфейсы:** `MotionPolicy.validate_trajectory(points, tcp_poses, current_joints) -> MotionCheck`; `MoveItBackend.plan_and_execute(goal: Transform, phase: str, held_object: str | None) -> MotionOutcome`; `MoveItBackend.stop() -> None`. Все планируемые траектории проходят один и тот же проверяющий слой.

- [x] Сначала проверить в установленном Jazzy/MoveItPy возможность задать `set_path_constraints`, семя IK из текущих суставов и исполнить траекторию через действующий контроллер Webots. Описать проверенные вызовы и ограничения в `docs/interfaces.md`. Если связка не исполняет план, не заменять её обычным `MoveToPose.action` без ограничений: зафиксировать блокирующую несовместимость и безопасный вариант реализации.
- [x] Написать тесты: J7 в `work` равен 0; границы −π/2 и +π/2 включительны; промежуточная точка за границей, скачок почти на π, наклон TCP >10°, несвежие `joint_states` и неактуальная планировочная сцена отвергаются до отправки контроллеру.
- [x] Реализовать проверку в `motion_policy.py`, затем MoveItPy backend с `current-state` IK seed, ограничением `joint7` и рабочего наклона, проверкой всей траектории, синхронизацией с `PlanningSceneBridge` и подтверждением достигнутой позы. Путь с удерживаемым объектом проверять вместе с прикреплённой геометрией.
- [x] Прогнать `pytest -q src/iiwa_llm_plan_executor/test/test_motion_policy.py src/iiwa_llm_plan_executor/test/test_moveit_backend.py`. В Webots выполнить один безопасный переход `work → над инструментом → work` и сохранить в QA журнале J7 по всем точкам и минимальный зазор; зафиксировать коммит.

### Задача 5. Контекст исполнения и навыки

**Файлы:** создать `src/iiwa_llm_plan_executor/iiwa_llm_plan_executor/{skill_context,skills}.py`, `test/test_skills.py`. Использовать `iiwa_experement/geometry.py`, `planning_scene.py` и протокол `/iiwa_experement/object_command`; не вызывать `reference_scenarios.py`.

**Интерфейсы:** `SkillContext` хранит текущий `scene_revision`, удерживаемый объект/инструмент, сохранённую безопасную позу подхода, фазу контакта и последнюю проверенную позу. `SkillRegistry.execute(node: PlanNode, context: SkillContext) -> SkillOutcome(event, details)`. Команды supervisor отправлять с `request_id` и подтверждать по следующему `object_state`.

- [x] Написать тесты фаз на фиктивном backend: `approach` идёт над целью и сохраняет высоту; `grab` без предварительного подхода выполняет подход, осевое опускание, захват и подъём; явные `approach/lower/lift` не дублируют движение. После `release` сначала осевой подъём, только затем горизонтальный уход или поворот.
- [x] Написать тесты инструментов: `grab(vacuum)` стыкуется кончиком `patron` к нужной точке; `release(vacuum)` из `work` **не двигает робот и не отделяет инструмент**; только после явного `transfer(vacuum,vacuum_socket)` навыки `release(vacuum,vacuum_socket)` и `dock_tool(vacuum,vacuum_socket)` опускаются в гнездо, отделяют и поднимаются. Отсутствие/занятость гнезда даёт отказ. `grab(cap)` без присоски не начинает захват: проверяется supervisor, контакт не выполняется, возвращается `grasp_failed`.
- [x] Написать тесты объектов: `transfer/align/release(cap,hatch_seat)` реально переносит и сажает люк; `dispense` движется вдоль заданного участка; `handoff_screw(screw_1,hole_1)` подносит винт и отпускает виртуальному человеку, не занимает отверстие установленным винтом. Неподдерживаемый тип цели отвергается.
- [x] Написать тесты `detect`: сначала осевой отвод и `work`, затем свежий верхний кадр + поза из supervisor; близкий кадр по настройке; просроченный кадр возвращает `failed`. `inspect_state` различает `empty`, `holding`, `placed`, `unknown`.
- [x] Реализовать навыки реестра из спецификации, используя общие геометрические примитивы. Каждый результат отображать в одно событие контракта; неизвестный исход не подменять успехом. Прогнать `pytest -q src/iiwa_llm_plan_executor/test/test_skills.py` и зафиксировать коммит.

### Задача 6. Движок графа и ROS узел

**Файлы:** создать `src/iiwa_llm_plan_executor/iiwa_llm_plan_executor/{engine,node}.py`, `test/test_engine.py`, `test/test_node.py`, `src/iiwa_msgs/srv/GetRunStatus.srv`; изменить `src/iiwa_msgs/CMakeLists.txt`; дополнить `docs/interfaces.md`.

**Интерфейсы:** `GraphEngine.submit(run_id: str, response_json: str, scene_revision: int) -> AdmissionResult`; `get_status(run_id: str) -> RunStatus`; `cancel(run_id: str) -> CancelResult`. ROS: `/iiwa_llm_plan_executor/submit` (`SetString`; JSON `{run_id, scene_revision, response}`), `/iiwa_llm_plan_executor/plan` (`String`; JSON `{run_id, scene_revision, response}`, исходный принятый граф без изменений, transient-local QoS, глубина 1), `/iiwa_llm_plan_executor/status` (`String`; события и итог), `/iiwa_llm_plan_executor/get_status` (`iiwa_msgs/srv/GetRunStatus`), `/iiwa_llm_plan_executor/cancel` (`SetString` с `run_id`).

- [x] Написать тесты: до готовности сцены запрос отклоняется; после задержки 3,0 с принимается; одновременно активен один `run_id`; повтор того же ID идемпотентен; новый ID при активном запуске отклоняется без второго движения. Принятый граф публикуется в `/iiwa_llm_plan_executor/plan` до первого движения, ровно один раз на `run_id`; отклонённый не публикуется; поздний подписчик получает последний принятый граф.
- [x] Написать тесты исполнения: каждое событие выбирает ровно одно ребро; `completed` завершается, `replan_required` даёт новый снимок, `operator_required` останавливается и уведомляет; `status=blocked` не отправляет движений; переходы и причина остановки видны в статусе.
- [x] Написать тесты отказов: потеря свежести сцены/суставов, неизвестный исход навыка, ошибка supervisor, столкновение, отмена и превышение предела шагов вызывают `stop`; удерживаемый инструмент/деталь не «исчезает» из статуса. Не продолжать по старой сцене после смены `scene_revision`, кроме изменений, сделанных и подтверждённых текущим графом.
- [x] Реализовать `GraphEngine` без ROS, затем сервис `GetRunStatus.srv`, ROS обёртку и машинно-читаемый журнал с `run_id`, ID узла, навыком, событием, следующей вершиной, снимком состояния и причиной терминала. Выполнение в отдельном рабочем потоке; колбэки статуса и отмены остаются доступными. Прогнать `pytest -q src/iiwa_llm_plan_executor/test/test_engine.py src/iiwa_llm_plan_executor/test/test_node.py`; зафиксировать коммит.

### Задача 7. HTTP и MCP через существующий сервер

**Файлы:** создать `src/iiwa_web/iiwa_web/planner.py`, `src/iiwa_web/test/test_planner.py`; изменить `src/iiwa_web/iiwa_web/main.py`.

**Интерфейс:** `GET /planner/scene`, `POST /planner/runs` с `{run_id, scene_revision, response}`, `GET /planner/runs/{run_id}`, `POST /planner/runs/{run_id}/cancel`. Тот же маршрутизатор включить в `_schema_app` до `FastMCP.from_fastapi` и в основной `app`; MCP получает соответствующие инструменты без второго сервера.

- [x] Написать тесты HTTP: корректный JSON снимка, принятие графа без ожидания его завершения, чтение статуса, отмена, ответ при недоступном ROS сервисе, конфликт второго запуска. Проверить, что MCP строится из того же маршрутизатора.
- [x] Реализовать адаптер через существующий `get_bridge().call_service`; он не интерпретирует узлы и не держит собственный `scene_revision`. В `POST /stop` добавить отмену активного графа до штатной команды `cobot/stop`.
- [x] Прогнать `pytest -q src/iiwa_web/test/test_planner.py src/iiwa_web/test/test_trajectory_admission.py`; зафиксировать коммит.

### Задача 8. Совместный запуск и документация пользователя

**Файлы:** создать `src/iiwa_llm_plan_executor/launch/planner.launch.py`, `src/iiwa_llm_plan_executor/config/planner.yaml`, `src/iiwa_llm_plan_executor/test/test_launch.py`, `src/iiwa_llm_plan_executor/doc/README.md`; обновить `setup.py` пакета для установки launch/config/doc и при необходимости `src/iiwa_experement/doc/README.md`.

**Интерфейс запуска:** `ros2 launch iiwa_llm_plan_executor planner.launch.py experiment_id:=20`. Launch включает `iiwa_experement/experiment.launch.py mode:=planner experiment_id:=20` и узлы сцены/графа. Уже включённый в `iiwa_bringup` сервер `iiwa_web` запускается **один раз** по `web.enabled` из `base_setting` (в текущем `cobot-setting.yaml` это `true`); новый launch не создаёт второй сервер. Для запуска только с ROS передать настройки с `web.enabled: false`. Значения задержки, таймаутов и J7/наклона берутся из конфигурации.

- [x] Написать тесты launch: в новом режиме есть scene context и executor, нет `reference_runner` и второго экземпляра `iiwa_web`, `simulate:=true`; `experiment_id` 1..20, недопустимый ID и неизвестный режим проверяются до запуска Webots.
- [x] Реализовать launch с готовностью `scene_ready` и задержкой; описать команды запуска, получение `scene_json`, передачу графа через ROS и HTTP, чтение `/iiwa_llm_plan_executor/plan` и `/iiwa_llm_plan_executor/status`, отмену, события/терминалы и то, что Qwen3 вызывается снаружи.
- [x] Прогнать `pytest -q src/iiwa_llm_plan_executor/test/test_launch.py` и `colcon build --base-paths src --symlink-install --packages-up-to iiwa_experement iiwa_scene_context iiwa_llm_plan_executor iiwa_web`; зафиксировать коммит.

### Задача 9. Проверки всей цепочки в Webots

**Файлы:** создать `src/iiwa_llm_plan_executor/doc/webots-qa.md`; по результатам исправить только затронутые файлы и тесты задач 1–8.

- [x] `source /opt/ros/jazzy/setup.bash`, затем `colcon build --base-paths src --symlink-install`, `source install/setup.bash`, `colcon test --base-paths src --packages-select iiwa_experement iiwa_scene_context iiwa_llm_plan_executor iiwa_web`, `colcon test-result --verbose`. Сохранить конкретные результаты.
- [ ] Запустить **одну и ту же** сцену 20 и последовательно передавать разные короткие графы: только `detect(cap)`; `grab(vacuum) → transfer(vacuum,work) → completed` с инструментом в `patron`; отдельным следующим графом `transfer(vacuum,vacuum_socket) → dock_tool(vacuum,vacuum_socket)`; после восстановления сцены — захват и установка люка; отдельно один винт к `hole_1`. Никакой опыт не должен автоматически выполнять полный эталон.
- [x] Проверить отказ: показанный граф люка без явного взятия присоски отклоняется **до движения**; ошибка захвата ведёт к `inspect_state` или `replan_required`; ложный/несвежий `scene_revision`, отмена и столкновение останавливают дальнейшие движения.
- [ ] В журнале подтвердить верхний подход, осевой спуск и подъём, фактическое перемещение люка/винта, сохранение инструмента в `patron` после завершения первого графа, возврат в гнездо только по второму явному графу, J7 внутри коридора на **всех** точках, отсутствие столкновений. Записать границы проверки: это Webots, а не проверка физического робота или качества Qwen3.
- [ ] После исправлений повторить затронутые тесты и `git diff --check`. Зафиксировать итоговый отчёт и передать пользователю команды запуска и ссылки на интерфейсы.

## Критерий готовности

Другой процесс может запросить актуальный `scene_json` через ROS/HTTP/MCP, получить граф от Qwen3 вне этой системы, передать его в ROS/HTTP/MCP, прочитать принятый граф из `/iiwa_llm_plan_executor/plan` и наблюдать каждый узел, событие, состояние и терминал в `/iiwa_llm_plan_executor/status`. Робот в Webots исполняет только данный граф с ограничениями TCP/J7 и подтверждённым захватом; при невозможности безопасного выполнения останавливается и сообщает причину. Система сравнения модели с эталоном остаётся отдельным следующим этапом.

