# Universal Grant Webots World Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Создать отдельный ROS 2 пакет `iiwa_experement`, из которого одной командой `ros2 launch` запускается универсальный мир Webots с детерминированной начальной сценой любого из 20 экспериментов.

**Architecture:** Новый пакет владеет шаблоном мира, конфигурацией 20 сценариев, генератором Webots `Solid`, спавнером и launch-файлом. Launch создаёт временную копию мира с абсолютными ссылками на установленные PROTO из `iiwa_description` и временный YAML с путём к этой копии. Затем он использует неизменённый `iiwa_bringup` для запуска Webots, робота iiwa 7, контроллеров и камеры; OBJ берёт из `iiwa_description`. Один запуск создаёт один эксперимент в свежем мире. Высокоуровневые навыки захвата, нанесения герметика и затяжки в этот план не входят; пакет готовит их начальные объекты и метаданные.

**Tech Stack:** Ubuntu 24.04, ROS 2 Jazzy, Webots R2025a, `ament_python`, Python 3, `rclpy`, `webots_ros2_msgs/srv/SpawnNodeFromString`, PyYAML, существующий `iiwa_bringup`.

**Spec:** `docs/superpowers/specs/2026-11-08-grant-experiments.md` и `docs/superpowers/specs/2026-11-08-grant-scenes.md` — версионируемые копии пользовательских `output/md/experements.md` и `output/md/experiment-scenes.md`. Задача 1 переносит их копии в новый пакет. Геометрические источники: `src/iiwa_description/worlds/iiwa.wbt`, `src/iiwa_description/objects/grant/`.

## Global Constraints

- Вся разработка ведётся в ветке **`experement`**, именно с таким написанием. Ветка создана при подготовке плана; перед изменениями исполнитель проверяет рабочее дерево и переключается на неё, не стирая существующие изменения.
- Весь новый код, запуск, мир, конфигурация, тесты и документация реализации находятся под `src/iiwa_experement/`. Исключение — существующие OBJ/MTL в `src/iiwa_description/objects/grant/`; их не копировать и не менять без отдельной необходимости. Этот файл плана расположен вне пакета как рабочее задание.
- Новый мир: `src/iiwa_experement/worlds/grant/iiwa-grant.wbt`. Прежнее предложение положить его в `iiwa_description/worlds/grant/` заменено новым требованием держать реализацию в отдельном пакете.
- `patron` постоянно установлен на роботе через `tool.active: patron`; рабочие инструменты размещаются отдельно и впоследствии вставляются в `patron`. Не создавать вторую модель `patron` в мире и не менять конфигурацию робота на `none`.
- Проверить также фактический `src/iiwa_description/urdf/tools/tool_active.xacro`: именно он сейчас включает `patron.xacro` в описание робота. Одного поля `tool.active` в YAML недостаточно, если этот файл изменился. Не править чужой Xacro в рамках пакета; при расхождении остановить интеграцию и сообщить о нём.
- Для всех экспериментов с лотком при запуске появляются **10 винтов**; это эксперименты 11–20. Сценарий может использовать только 1, 2 или 4 из них. Винты взаимозаменяемы; оценка плана не зависит от выбора конкретного номера.
- Мир и объекты используют метры и мировую систему Webots. Координаты, ориентации, наборы объектов и смещения брать из `experiment-scenes.md`; не угадывать по фотографиям. Фотографии в `output/reference_photo/grant/` служат только визуальной сверкой.
- Не восстанавливать старые удалённые `body_hatch*.wbt` и `GrantToolRack.proto`. Стойка строится в новом пакете из `tool-magizine.obj` и отдельных объектов инструментов.
- Не добавлять исполнителей `dispense`, `press`, `seat_screw`, `tighten`, управление захватом и динамическую фиксацию люка в рамках этого плана. В документации явно отделить готовую начальную сцену от последующего исполнения навыков.
- В конце каждого этапа выполнять указанные проверки и делать отдельный коммит с кратким префиксом `feat:`, `test:` или `docs:`.

## Review Focus

1. Номер вне `1..20` либо нечисловой ввод должен остановить launch **до** запуска Webots. Проверяет задача 6.
2. Отсутствующий OBJ/MTL или неразрешённый PROTO после изолированной установки пакета должен давать явную ошибку, а не пустой объект. Проверяют задачи 2, 4 и 6.
3. Повторяющиеся имена объектов либо сцена с 9/11 винтами не должны проходить проверку сценария. Проверяет задача 3.
4. Тайм-аут ответа supervisor не должен автоматически повторять spawn: первый запрос мог быть выполнен, и повтор создаст дубликат. Проверяет задача 5.
5. Запуск не должен одновременно включить старый случайный `object_spawner` или робота без `patron` в сгенерированном URDF. Проверяет задача 6.

---

## Карта файлов

Создать только внутри `src/iiwa_experement/`:

| Файл или папка | Ответственность |
| --- | --- |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/iiwa_experement` | ROS 2 пакет `ament_python`; установка Python, launch, YAML, WBT и MD |
| `iiwa_experement/__init__.py` | Маркер Python пакета |
| `iiwa_experement/scenario.py` | Чтение `scenes.yaml`, проверка ID и инвариантов, типизированный `ScenarioSpec` |
| `iiwa_experement/webots_nodes.py` | Преобразование `ScenarioSpec` в детерминированный список Webots `Solid` со стабильными именами |
| `iiwa_experement/scene_spawner.py` | ROS нода: вызов supervisor spawn, обработка ошибок, сигнал готовности |
| `iiwa_experement/runtime_config.py` | Подготовка временных WBT/YAML с установленными путями PROTO, миром, `patron` и отключённым случайным спавном |
| `config/scenes.yaml` | Единственный машинный источник координат, наборов объектов и метаданных отказов для 20 опытов |
| `worlds/grant/iiwa-grant.wbt` | Исходный шаблон универсального мира из `iiwa.wbt`: стол, ограждения, свет; без робота и деталей |
| `launch/experiment.launch.py` | Команда `ros2 launch iiwa_experement experiment.launch.py experiment_id:=N` |
| `doc/experements.md`, `doc/experiment-scenes.md` | Версионируемые копии двух MD спецификаций из `output/md/` |
| `doc/README.md` | Запуск, параметры, проверка готовности, границы реализации |
| `test/test_package_layout.py`, `test/test_scenario.py`, `test/test_webots_nodes.py`, `test/test_scene_spawner.py`, `test/test_launch.py`, `test/test_world.py`, `test/test_runtime_config.py` | Проверки установки пакета, данных, генерации узлов, путей, отказов и запуска |

Перед работой прочитать: инструкции AGENTS.md из задания; `docs/superpowers/specs/2026-11-08-grant-experiments.md`; `docs/superpowers/specs/2026-11-08-grant-scenes.md`; `src/iiwa_description/worlds/iiwa.wbt`; `src/iiwa_description/resource/protos/CollaborativeRobotTable.proto`; `src/iiwa_description/resource/protos/WorkspaceLimiter.proto`; `src/iiwa_description/objects/grant/`; `src/iiwa_description/urdf/tools/tool_active.xacro`; `src/iiwa_config/config/setting.yaml`; `src/iiwa_config/config/tools.yaml`; `src/iiwa_bringup/launch/iiwa.launch.py`; `src/iiwa_bringup/launch/supported/webots_spawn.launch.py`; `src/iiwa_utils/iiwa_utils/object_spawner.py`. При необходимости визуально сверить `output/reference_photo/grant/`, но помнить, что `output/` игнорируется Git.

### Task 1: Ветка, пакет и версионируемые спецификации

**Files:** создать `src/iiwa_experement/{package.xml,setup.py,setup.cfg,resource/iiwa_experement,iiwa_experement/__init__.py,doc/experements.md,doc/experiment-scenes.md}`.

**Interfaces:** производит установленный пакет `iiwa_experement` и доступные через его share-директорию MD файлы; остальные задачи используют эту директорию через `ament_index_python`.

- [ ] **Step 1: Проверить состояние Git и ветку.** Выполнить `git status --short` и `git branch --show-current`; при необходимости `git switch experement`. Не сбрасывать существующие удаления старых миров и прототипа. Проверить `git branch --show-current` → `experement`.
- [ ] **Step 2: Создать минимальный `ament_python` пакет.** В `package.xml` указать прямые зависимости `rclpy`, `ament_index_python`, `webots_ros2_msgs`, `std_srvs`, `python3-yaml`, `iiwa_description`, `iiwa_bringup`, `iiwa_config`, `launch`, `launch_ros`; тестовую зависимость `python3-pytest`. В `setup.py` установить существующие `doc/` в `share/iiwa_experement`; добавлять `worlds/`, `config/`, `launch/` по мере создания этих папок. Зарегистрировать `scene_spawner = iiwa_experement.scene_spawner:main` после задачи 5.
- [ ] **Step 3: Скопировать спецификации без изменения смысла** из `docs/superpowers/specs/2026-11-08-grant-experiments.md` и `docs/superpowers/specs/2026-11-08-grant-scenes.md` в `doc/experements.md` и `doc/experiment-scenes.md`. Проверить совпадение содержимого; копии внутри пакета устанавливаются вместе с ним.
- [ ] **Step 4: Проверить регистрацию пакета.** `source /opt/ros/jazzy/setup.bash && colcon list --base-paths src | rg '^iiwa_experement\s'` → ровно одна строка; после появления всех устанавливаемых ресурсов выполнить сборку `colcon build --base-paths src --packages-up-to iiwa_experement --symlink-install`.
- [ ] **Step 5: Коммит.** `git add src/iiwa_experement` и `git commit -m "feat: scaffold iiwa_experement package"`.

### Task 2: Универсальный базовый мир

**Files:** создать `worlds/grant/iiwa-grant.wbt`, `test/test_world.py`.

**Interfaces:** мир открывается Webots по `pkg://iiwa_experement/worlds/grant/iiwa-grant.wbt`; объекты эксперимента добавляет задача 5, робот — существующий bringup.

- [ ] **Step 1: Написать тест `test_world_matches_base_layout`.** Проверить `basicTimeStep 32`, стол `translation 0 0 -0.007`, оба `WorkspaceLimiter`, отсутствие статического робота, люка, корпуса и винтов. Проверить, что шаблон содержит только две известные локальные ссылки на `CollaborativeRobotTable.proto` и `WorkspaceLimiter.proto`, которые задача 6 заменит на абсолютные перед запуском. Добавить `test/test_package_layout.py::test_expected_resources_are_installed`, который после сборки проверяет наличие WBT, `scenes.yaml`, launch и двух MD в share-директории; пока будущие ресурсы не созданы, проверять в нём только уже готовые файлы.
- [ ] **Step 2: Запустить тест, убедиться в ожидаемом отказе** из-за отсутствующего мира.
- [ ] **Step 3: Скопировать основу `iiwa.wbt` в новый файл и добавить его в `setup.py`.** В исходном шаблоне использовать относительные пути `../../../iiwa_description/resource/protos/CollaborativeRobotTable.proto` и `../../../iiwa_description/resource/protos/WorkspaceLimiter.proto`: они работают из `src`, но **не** из изолированного `install/`; задача 6 заменит их при запуске. Убрать неиспользуемый `Object.proto`. Сохранить фон, свет, стол, ограждения, `basicTimeStep`; выбрать точку обзора, показывающую всю рабочую зону. Установить ненулевое трение `TableMaterial` (ориентир `0.6` из прежней grant-сцены), не менять исходный `iiwa.wbt`.
- [ ] **Step 4: Запустить тест и сборку пакета.** Тест проходит; после установки `ros2 pkg prefix iiwa_experement` показывает пакет, а шаблон мира находится в `share/iiwa_experement/worlds/grant/`.
- [ ] **Step 5: Коммит.** `git commit -m "feat: add universal grant Webots world"`.

### Task 3: Данные 20 сценариев и строгая проверка

**Files:** создать `config/scenes.yaml`, `iiwa_experement/scenario.py`, `test/test_scenario.py`.

**Interfaces:** `load_scenario(experiment_id: int, manifest_path: pathlib.Path) -> ScenarioSpec`. `ScenarioSpec` содержит ID, позы стапеля/корпуса/стойки, состояние и позу люка (`absent|free|installed`), необязательные позы лотка и 10 винтов, перечень рабочих инструментов, метаданные отказов. Задача 4 использует только этот объект, а не читает YAML повторно.

- [ ] **Step 1: Написать тесты для всех ID 1–20.** Проверить наборы: 1–5 — свободный люк, нет лотка, только присоска; 6–9 — нет люка и лотка, герметик; 10 — свободный люк, герметик и присоска; 11–15 — установленный люк и ровно 10 винтов; 16–20 — свободный люк и ровно 10 винтов. Проверить особые позы № 2, 3, 5, 19 и метаданные отказов № 4, 5, 9, 15, 20 по `doc/experiment-scenes.md`.
- [ ] **Step 2: Добавить отрицательные тесты.** ID `0`, `21`, неизвестное имя инструмента, 9/11 винтов, повторяющееся имя, винт вне лотка, несовпадающие XY корпуса и стапеля должны давать `ValueError` с понятным сообщением.
- [ ] **Step 3: Запустить тесты и увидеть ожидаемые отказы** до реализации загрузчика.
- [ ] **Step 4: Реализовать компактный YAML манифест и `load_scenario`, добавить YAML в `setup.py`.** Хранить общую геометрию один раз; в эксперименте указывать только выбор объектов и отличия. Десять винтов задавать сеткой/позами из спецификации; для № 19 переносить **все десять** вместе с лотком. Никакого случайного расположения. Состояния ошибок в YAML — метаданные для будущего исполнителя, они не меняют начальные позы, кроме уже установленного люка в опытах 11–15.
- [ ] **Step 5: Запустить `python3 -m pytest src/iiwa_experement/test/test_scenario.py -q` после sourcing ROS.** Ожидается 0 ошибок; проверить, что данные для 20 опытов читаются из установленного YAML.
- [ ] **Step 6: Коммит.** `git commit -m "feat: define deterministic grant scenarios"`.

### Task 4: Генератор Webots-объектов

**Files:** создать `iiwa_experement/webots_nodes.py`, `test/test_webots_nodes.py`.

**Interfaces:** `build_nodes(spec: ScenarioSpec, asset_root: pathlib.Path) -> list[str]`. Возвращает Webots VRML строки в порядке: стапель → корпус → стойка → рабочие инструменты → лоток → 10 винтов → люк. `asset_root` равен `get_package_share_directory('iiwa_description')/objects/grant`.

- [ ] **Step 1: Написать тесты узлов.** Проверить реальные URL OBJ, уникальные `name`, отсутствие `GrantToolRack.proto`, отсутствие дублированного `patron`, правильные позы и число винтов для сценариев 1, 6, 11, 16, 19. В № 19 у каждого винта должен быть тот же сдвиг, что у лотка. Для 20 ID проверить, что каждый требуемый OBJ существует.
- [ ] **Step 2: Написать тесты геометрии.** Проверить преобразование вертикальной оси OBJ Y в Z Webots, повороты 45°/135°/60° люка, масштаб `10` **только** для `germetic-tool.obj`, нижнюю точку свободного люка на `Z=0.793`, установленного люка на верхней поверхности корпуса и 10 различных поз винтов внутри лотка.
- [ ] **Step 3: Запустить тесты и увидеть ожидаемые отказы.**
- [ ] **Step 4: Реализовать генератор.** Использовать отдельный `Solid` для стойки из `tool-magizine.obj` и отдельный `Solid` для каждого инструмента; стабильные имена `grant_fixture`, `grant_body`, `grant_rack`, `grant_cap`, `grant_tray`, `grant_vacuum`, `grant_magnetic_tool`, `grant_sealant_tool`, `grant_torque_tool`, `grant_screw_01`…`grant_screw_10`. Добавить простые проверенные `boundingObject` для физических объектов. Стапель, корпус, стойка и лоток фиксированы. Свободный люк имеет `Physics`; установленный зафиксирован. Для винтов и инструментов явно документировать начальное состояние «лежит в лотке/гнезде» и границу будущего механизма захвата; не выдавать статическую геометрию за реализованный захват.
- [ ] **Step 5: Запустить тесты и проверить установленный asset root.** `python3 -m pytest src/iiwa_experement/test/test_webots_nodes.py -q` → 0 ошибок; ни один URL не указывает в `output/` или в исходную директорию вместо установленного пакета.
- [ ] **Step 6: Коммит.** `git commit -m "feat: generate grant Webots objects"`.

### Task 5: Детерминированный спавнер и готовность сцены

**Files:** создать `iiwa_experement/scene_spawner.py`, `test/test_scene_spawner.py`; дополнить `setup.py` console script.

**Interfaces:** `scene_spawner` принимает ROS параметр `experiment_id: int` и путь к `scenes.yaml` (по умолчанию установленный файл); вызывает `/Ros2Supervisor/spawn_node_from_string` по одному узлу; после всех успешных ответов оставляет доступным сервис `/iiwa_experement/scene_ready` типа `std_srvs/srv/Trigger` (`success=true`, сообщение с ID). До завершения и при отказе сервис не сообщает готовность.

- [ ] **Step 1: Написать тест успешного порядка вызовов.** Подставить fake client; сверить порядок и число отправленных строк с `build_nodes`; готовность наступает только после последнего `success=true`.
- [ ] **Step 2: Написать тесты отказов.** Нет supervisor за заданный тайм-аут, `success=false`, исключение и тайм-аут ответа должны завершаться ошибкой с ID последнего объекта. При тайм-ауте **не повторять** запрос автоматически. Если Webots уже содержит то же имя, не скрывать отказ дубликата.
- [ ] **Step 3: Запустить тесты и увидеть ожидаемые отказы.**
- [ ] **Step 4: Реализовать ноду по образцу `iiwa_utils/object_spawner.py`, но без случайного выбора.** Использовать `get_package_share_directory` для YAML и OBJ; дождаться supervisor service; отправлять узлы последовательно; логировать эксперимент, каждый ID объекта и окончательную готовность. Оставить ноду активной для сервиса `scene_ready`.
- [ ] **Step 5: Запустить тесты и коммит.** `python3 -m pytest src/iiwa_experement/test/test_scene_spawner.py -q` → 0 ошибок; `git commit -m "feat: spawn deterministic grant scene"`.

### Task 6: Запуск одного эксперимента командой ROS 2

**Files:** создать `iiwa_experement/runtime_config.py`, `launch/experiment.launch.py`, `test/test_runtime_config.py`, `test/test_launch.py`, `doc/README.md`.

**Interfaces:** `prepare_runtime_files(base_setting: pathlib.Path, world_template: pathlib.Path, description_share: pathlib.Path, output_dir: pathlib.Path) -> tuple[pathlib.Path, pathlib.Path]` возвращает `(world_path, setting_path)`. Команда `ros2 launch iiwa_experement experiment.launch.py experiment_id:=N` работает для `N=1..20`; необязательные `rviz:=true|false`, `scene_config:=<путь к YAML>` и `base_setting:=<путь к YAML>`. Launch передаёт временный `setting_path` в `iiwa_bringup/launch/iiwa.launch.py` с `simulate:=true` и запускает локальный `scene_spawner` с тем же ID и `scene_config`.

- [ ] **Step 1: Написать тесты `validate_experiment_id(raw: str) -> int`.** `1`, `20` проходят; `0`, `21`, `abc`, пустая строка вызывают ошибку до создания действий Webots. Проверить, что launch включает именно `iiwa_bringup` и один `scene_spawner`, а `scene_config` передаётся ему без изменения.
- [ ] **Step 2: Написать `test_runtime_config.py`.** На временном каталоге проверить, что `prepare_runtime_files` заменяет обе локальные `EXTERNPROTO` на абсолютные существующие пути из `description_share`, не меняет внешние `https://` PROTO, пишет отдельный WBT и YAML. В YAML проверить путь на временный WBT, `digital_twin.webots.objects.enabled: false`, `count: 0`, `tool.active: patron`, сохранение камеры `d455_top`, позы робота `-0.25 0 0.79` и контроллеров. Добавить отрицательные случаи отсутствующего PROTO и отсутствующего шаблона. Проверить `iiwa_utils.setting_loader.build_settings(..., check_files=True)` на готовом YAML; также проверить включение `patron.xacro` в `tool_active.xacro` и наличие `patron` в сгенерированном URDF.
- [ ] **Step 3: Запустить тесты и увидеть ожидаемые отказы.**
- [ ] **Step 4: Реализовать `runtime_config` и launch, добавить launch в `setup.py`.** По умолчанию читать установленный `iiwa_config/config/setting.yaml`; получить `description_share` через `get_package_share_directory('iiwa_description')`; подставить абсолютные пути PROTO в копию мира и записать оба временных файла в отдельную директорию запуска. Принять только один ID и проверить его в `OpaqueFunction` до включения Webots; передать ID в спавнер. Очистить временную директорию при завершении launch. Не запускать legacy `object_spawner`; не менять FRI-ветку существующего bringup и не запускать физического робота. Не подменять `patron` пустым фланцем.
- [ ] **Step 5: Написать `doc/README.md` и завершить `test_package_layout.py`.** Дать команды сборки, запуска № 1, 6, 11, 16, 19, 20, проверки `/iiwa_experement/scene_ready`, остановки и выбора другого ID новым запуском. Объяснить: файл мира расположен в новом пакете, OBJ остаются в `iiwa_description`, 10 винтов всегда присутствуют в сценах 11–20; исполнение навыков и привязка инструмента к `patron` ещё не входят в пакет. После сборки тест установки должен проверить все WBT, YAML, launch и MD в share-директории.
- [ ] **Step 6: Запустить тесты и коммит.** `python3 -m pytest src/iiwa_experement/test/test_runtime_config.py src/iiwa_experement/test/test_launch.py -q` → 0 ошибок; `git commit -m "feat: launch grant experiment by id"`.

### Task 7: Интеграционная проверка и передача

**Files:** при необходимости исправлять только файлы `src/iiwa_experement/`; обновить `doc/README.md` наблюдениями.

**Interfaces:** установленный пакет запускает 20 сцен из одного мира, логирует выбранный ID и сообщает готовность лишь после успешного спавна всех объектов.

- [ ] **Step 1: Сборка и пакетные тесты.** Из корня: `source /opt/ros/jazzy/setup.bash`, `colcon build --base-paths src --packages-up-to iiwa_experement --symlink-install`, `source install/setup.bash`, `colcon test --base-paths src --packages-select iiwa_experement`, `colcon test-result --verbose`. Исправить ошибки нового пакета до перехода дальше.
- [ ] **Step 2: Проверить все 20 конфигураций без Webots.** Тесты должны подтвердить состав, координаты, уникальные имена, наличие 10 винтов в опытах 11–20 и отсутствие винтов в 1–10. Сверить по строкам `doc/experiment-scenes.md`.
- [ ] **Step 3: Проверить Webots запуск в графической среде.** По отдельности запустить `ros2 launch iiwa_experement experiment.launch.py experiment_id:=1`, затем 6, 11, 16, 19, 20. Дождаться `scene_ready`; осмотреть положение люка, стапеля, лотка, десяти винтов, инструментов и видимость верхней камеры. После каждого опыта завершить запуск и начать свежий мир. Если графический дисплей недоступен, записать это как непроверенный пункт, не объявлять визуальную проверку пройденной.
- [ ] **Step 4: Проверить ошибки запуска.** `experiment_id:=0` и `experiment_id:=21` должны завершиться до Webots. Создать временную копию YAML с девятью винтами вместо десяти и передать через `scene_config:=<путь>`: спавнер должен дать отказ без объявления готовности. Штатные файлы после проверки не менять. Отсутствующий OBJ проверяется тестом задачи 4 с временным `asset_root`.
- [ ] **Step 5: Финальная проверка границ.** `git diff --check`; `git status --short`; `git branch --show-current` → `experement`. Проверить, что изменённые файлы реализации находятся только в `src/iiwa_experement/`, кроме ранее существовавших удалений; исходный `iiwa.wbt`, OBJ и `cobot-setting.yaml` не переписаны. Отдельно сообщить, что старый `src/iiwa_utils/test/test_body_hatch_worlds.py` ссылается на удалённые миры/PROTO и не является тестом нового пакета.
- [ ] **Step 6: Коммит завершения и отчёт.** Если потребовались интеграционные исправления, сделать отдельный коммит. В отчёте указать команды и результаты, какие Webots сценарии просмотрены, какие проверки не удалось выполнить, и точную команду запуска любого `N`.

## Приёмка

План выполнен, когда из установленного рабочего пространства работает `ros2 launch iiwa_experement experiment.launch.py experiment_id:=N` для каждого `N` от 1 до 20; Webots загружает один и тот же `iiwa-grant.wbt`, а спавнер создаёт соответствующий набор объектов и выдаёт готовность. В опытах 11–20 лоток содержит 10 винтов, `patron` находится на роботе во всех опытах, и нужные рабочие инструменты находятся в отдельных гнёздах стойки. Реализация и её тесты лежат в `src/iiwa_experement/`; источники OBJ/MTL остаются в `iiwa_description`.
