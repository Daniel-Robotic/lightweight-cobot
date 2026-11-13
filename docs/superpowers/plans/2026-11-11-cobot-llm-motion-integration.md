# Интеграция графов LLM в штатный запуск `cobot`

> Для код-агента. Реализуй задачи по порядку в ветке `experement` по текущей рабочей копии. Здесь описаны изменения и критерии приёмки.

После `cobot run` робот должен принимать готовый JSON-граф LLM и перемещать установленный оператором инструмент в нужную точку. Обычный Webots использует `digital_twin.webots.world`, а запуск контроллера использует FRI. Опыты Grant по-прежнему запускаются в Webots через существующие launch-файлы с явным номером опыта. Внешний клиент вызывает LLM и отправляет готовый граф.

| Запуск | Исполнитель | Источник контекста для LLM |
| --- | --- | --- |
| `cobot run local` или `cobot run docker` → «Симулятор Webots» | Базовые перемещения через штатный MoveIt | `iiwa_scene_context` публикует снимок базовой сцены |
| `cobot run local` или `cobot run docker` → «Контроллер» | Те же перемещения через штатный MoveIt и FRI при включении в YAML | Именованные позиции и телеметрия робота через штатные интерфейсы |
| `ros2 launch iiwa_llm_plan_executor planner.launch.py experiment_id:=20` | Существующие навыки Grant | Существующий `scene_json` от Grant Supervisor |

Для первой версии предлагаю поддержать именованную позицию и явную TCP-позу, включать исполнение графов на физическом роботе отдельной настройкой и покрыть обе среды `cobot run`. Навыки захвата, переноса объектов, стыковки инструмента и технологических операций входят только в профиль Grant. На физическом роботе `scene_json` пока отсутствует. Имя инструмента, его TCP, коллизионная модель и нагрузка берутся из действующей конфигурации робота и контроллера.

## 1. Подключить узлы к штатному запуску

- [ ] Добавить в `cobot-setting.yaml` и `src/iiwa_config/config/setting.yaml` одинаковый блок `llm_planner`. В блок входят `enabled: true`, `hardware_enabled: false` и `velocity_scale: 0.10`. Значение масштаба `0.10` служит начальным значением для Webots. Для физического прогона оператор задаёт согласованное значение. В `src/iiwa_utils/iiwa_utils/setting_loader.py` добавить типизированное чтение блока, строгие булевы значения и конечный масштаб в диапазоне `(0, 1]`. Отсутствие блока в старом YAML означает выключенную интеграцию. При выключенном `enabled` штатный запуск работает независимо от `hardware_enabled`.
- [ ] Создать `src/iiwa_llm_plan_executor/launch/cobot.launch.py` с аргументами `setting`, `simulate`, `rviz`. Включить `iiwa_bringup/iiwa.launch.py` один раз с теми же аргументами. При `enabled: true` и `simulate: true` добавить `scene_context` и `llm_plan_executor` в базовом режиме с `use_sim_time: true`. При `simulate: false` и двух включённых флагах добавить только исполнитель с `use_sim_time: false`. В обоих режимах использовать настройки и пути штатного bringup. Проверять конфигурацию до создания процессов.
- [ ] Использовать общий конструктор команды запуска в `cobot/commands/run.py` для `_run_local` и `_run_docker`. Сохранить меню, подключение ROS overlay, передачу `simulate`, монтирование `cobot-setting.yaml` и остановку дочернего дерева. Для устаревшей сборки вывести команду `cobot rebuild --symlink-install` или `cobot docker-setup` и вернуть ошибку запуска.
- [ ] Объявить зависимости нового launch в `src/iiwa_llm_plan_executor/package.xml` и проверить установку через `setup.py`. Оставить направление зависимости от нового launch к `iiwa_bringup`. Обратная зависимость образует цикл `iiwa_bringup → iiwa_llm_plan_executor → iiwa_experement → iiwa_bringup`. Существующие `planner.launch.py` и `experiment.launch.py` продолжают включать низкоуровневый bringup и свои узлы Grant.
- [ ] Тесты в `cobot/test/` и `src/iiwa_llm_plan_executor/test/` покрывают матрицу Webots/FRI × флаги, старый и неверный YAML, аргументы local/Docker и единственный экземпляр контроллера, MoveIt и `iiwa_web`. При запуске обычного Webots отсутствуют процессы `scene_spawner`, `grant_supervisor` и `reference_runner`. Для существующего launch Grant сохранить прежние тесты, включая отказ при `simulate:=false`.

## 2. Принимать базовые графы без сцены Grant

- [ ] В `src/iiwa_llm_plan_executor/iiwa_llm_plan_executor/{node,engine,validator,skills}.py` выделить режим `base` с собственными проверками и двумя навыками. Сохранить общий разбор JSON, публикацию оригинала, `run_id`, повторную отправку, один активный запуск, статус, журнал и отмену. Сценозависимые проверки ревизии, владения и готовности оставить в режиме Grant. В `base` определять готовность по актуальному полному `/joint_states`, готовности штатного сервера движения и активному контроллеру. Проверять состояние перед запуском графа и перед каждым шагом. Выход FRI из рабочего состояния, потеря обратной связи или недоступный контроллер прерывают граф и запрещают следующий шаг. Учитывать уже реализованную обработку `COMMANDING_ACTIVE` в `iiwa_controller`.
- [ ] Сохранить формат `status`, `entry`, `plan`, `transitions` и интерфейс `/iiwa_llm_plan_executor/submit_plan` (`iiwa_msgs/srv/SubmitPlan`). Для `move_to_named` поле `target` задаёт имя `group_state` из активного SRDF для `planning.planning_group`. Для `move_to_pose` требовать ровно шесть чисел `x`, `y`, `z` в метрах и `a`, `b`, `c` в радианах в `params`. Углы задаются в порядке ZYX по соглашению KUKA ABC, фрейм фиксирован `base_link`, TCP-линк берётся из `planning.pose_link`. Для обоих навыков `object` пустой. У `move_to_pose` поле `target` также пустое. Имена, числа, допустимые поля и переходы проверить для всех узлов при приёме. Отклонять неизвестные команды, ссылки на объекты Grant, `NaN`/`Inf`, внешние фреймы и попытки переопределить масштаб скорости. Публиковать результат шага `success` или `motion_failed` по результату сервера. Для `motion_failed` разрешать только переход в `operator_required` или `replan_required`. При отсутствии такого перехода завершать граф как `failed` с причиной.
- [ ] Передавать TCP-цель в штатный action `cobot/move_to_pose` (`iiwa_msgs/action/MoveToPose`). Для именованного движения добавить отменяемый `cobot/move_to_named_pose` (`iiwa_msgs/action/MoveToNamed`) в `src/iiwa_planning/scripts/move_to_pose_server.py` и `src/iiwa_msgs`. Вынести существующее планирование именованной цели из `cobot/move_to_named` в общий участок, чтобы оба интерфейса использовали SRDF, Pilz PTP, лимиты и `MotionSerialiser`. Штатный сервис сохраняет контракт. Для TCP-цели применять настроенный планировщик и масштаб, согласованные с MoveIt. Успех шага требует успешного результата action, включая выполнение траектории контроллером.
- [ ] Подключить отмену графа к goal handle MoveIt и штатной остановке. Проверить окно между отправкой цели и получением goal handle, потерю контроллера и повторный submit после отмены. `POST /stop` должен остановить граф, отправленный через ROS, HTTP или MCP. Текущий `_active_run_id` в `src/iiwa_web/iiwa_web/planner.py` учитывает только HTTP-запросы. Источник активного запуска и его отмены должен находиться в executor. Штатный `/cobot/stop` должен инвалидировать активную операцию и ожидающие шаги графа. В базовом режиме графы и обычные запросы MoveIt проходят через одну сериализацию. Проверить, что прямой HTTP-ввод траекторий не перебивает активный граф.
- [ ] Добавить тесты базового графа с подменой ROS action-сервера. Покрыть успешные именованную и TCP-цели, отсутствие `scene_json` на FRI, отказ при неизвестном имени и некорректных координатах, `not_ready` при устаревших суставах, ошибку контроллера, отмену до принятия цели и в процессе исполнения, дубликат и параллельный запуск. Прогнать существующие тесты Grant и `test_motion_priority.py`.

## 3. Выдавать честный `scene_json` в базовом Webots

- [ ] В `src/iiwa_scene_context/iiwa_scene_context/{node,snapshot}.py` добавить режим `base`. Источник актуальности базового снимка — `/joint_states` и TF из `base_link` к настроенному TCP, если трансформ доступен. Публиковать тип базового профиля, метку времени, наблюдаемое состояние, доступные базовые навыки и имена поз из активного SRDF. В пустом мире набор наблюдаемых объектов задачи пуст. У базового снимка нет номера опыта. Отсутствующие данные о камерах, объектах и установленном инструменте обозначать явно. Сцена без объектов задачи не подтверждает отсутствие препятствий для MoveIt.
- [ ] Сохранить `/iiwa_scene_context/get_scene`, QoS `RELIABLE`/`TRANSIENT_LOCAL` для `/iiwa_scene_context/scene_json` и действующий формат Grant. Базовый контекст не подписывается на `/iiwa_experement/object_state` и не ждёт `/iiwa_experement/scene_ready`. Возвращать понятный отказ до первого валидного состояния и при его устаревании. Для FRI сервис сцены не запускается, а HTTP/MCP сообщает, что `scene_json` доступен только в Webots. Маршруты отправки, статуса и отмены работают при включённом `web.enabled` в обоих режимах.
- [ ] Тесты проверяют свежесть и содержимое пустой сцены без Grant, достоверность источников полей и неизменный контракт сцены опыта. Проверить, что пропажа базового `scene_json` сама по себе не блокирует допустимую команду при наличии актуальной телеметрии и готового MoveIt.

## 4. Собрать и проверить запуск из установленных пакетов

- [ ] В обоих `docker/jazzy/{ros-iiwa7,ros-iiwa7-webots}/Dockerfile` добавить исходники `iiwa_experement`, `iiwa_scene_context` и `iiwa_llm_plan_executor` в контекст перед `rosdep` и `colcon`. Проверить зависимости из `package.xml` и доступность launch, конфигов и ROS-интерфейсов в release-образах после удаления `src`. Docker для контроллера не требует приложения Webots. Через `cobot docker-setup` собрать оба образа. Проверить `cobot run docker` на новом Webots-образе и стенде FRI с имитацией аппаратуры.
- [ ] С ROS 2 Jazzy запустить `cobot rebuild --symlink-install`, подключить `install/setup.bash` и выполнить `colcon test --base-paths src` и `colcon test-result --verbose`. Для общего запуска `pytest` по нескольким пакетам использовать `--import-mode=importlib` из-за совпадающих имён `test_launch.py`. Сверить результаты с исходным состоянием рабочей копии, включая незакоммиченный `src/iiwa_msgs/srv/SubmitPlan.srv`.
- [ ] В обычном Webots запустить `cobot run local` → «Симулятор Webots». Проверить один набор штатных узлов и базовый снимок через `ros2 service call /iiwa_scene_context/get_scene std_srvs/srv/Trigger '{}'`. Сначала отправить отклоняемый граф и убедиться, что контроллер не получил цель. Затем отправить разрешённую именованную цель и допустимую TCP-цель, выбранную для установленной модели и сцены. Для обеих сверить `/plan`, `/status`, фактическую позу и `completed`. Отдельно проверить отмену, общий stop и чистое завершение по Ctrl-C. Повторить на Docker. Проверить, что ручной запуск Grant с `experiment_id:=20` отдаёт прежнюю сцену и принимает прежний граф.
- [ ] Для FRI сначала проверить ветку запуска с имитацией action-сервера и контроллера либо с тестовым FRI-стендом из `src/iiwa_controller/test/`. Убедиться, что нет Webots и сервиса сцены, выключенная настройка исключает executor, а при включении граф до готовности возвращает `not_ready`. Физическую приёмку выполнять с оператором после проверки сети, активного режима FRI, лимитов, модели инструмента, Load Data и рабочего пространства. Начать с одной заранее проверенной именованной позиции на согласованной скорости. В отчёте указать отдельно результаты Webots, FRI-имитации и физической проверки.
- [ ] Описать операторский запуск и примеры графов в `src/iiwa_llm_plan_executor/doc/README.md`, контракт базовой сцены в `src/iiwa_scene_context/docs/interfaces.md` и настройки в `doc/lwc-doc/docs/getting-started/configuration{,.en}.md`.

Пример тела `POST /planner/runs` для Webots. Перед запуском проверьте, что `work` есть в активном SRDF и применим к текущей сцене.

```json
{
  "run_id": "base-webots-1",
  "response": {
    "status": "ready",
    "entry": "go",
    "plan": [{
      "id": "go",
      "skill": "move_to_named",
      "target": "work",
      "transitions": [
        {"event": "success", "next": "completed"},
        {"event": "motion_failed", "next": "operator_required"}
      ]
    }]
  }
}
```

## Ссылки для реализации

- Проектные контракты по запуску, API и остановке. [Deployment](../../../openspec/specs/deployment/spec.md), [External API](../../../openspec/specs/external-api/spec.md), [Motion arbitration](../../../openspec/specs/motion-arbitration/spec.md).
- Документация ROS 2 Jazzy по вложенным launch-файлам и зависимостям. [Launch](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.rst), [rosdep](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Intermediate/Rosdep.rst).
