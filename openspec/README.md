# Спецификация Lightweight Cobot

Эти базовые OpenSpec-спецификации описывают поведение, обнаруженное в исходном коде и конфигурации на 5 ноября 2026 года. `specs/` задаёт текущий контракт системы; будущие изменения следует оформлять в `changes/` и после реализации переносить в основные спецификации через архивирование OpenSpec.

## Границы системы

```text
CLI cobot / ROS actions / HTTP и MCP / Foxglove и RViz
                ↓
        iiwa_bringup + конфигурация
                ↓
       MoveIt 2 + motion server
                ↓
    joint_trajectory_controller
             ↙           ↘
    Webots ros2_control   FRI hardware interface → Sunrise Cabinet
```

Физическое движение опирается на штатные функции безопасности контроллера KUKA и наблюдение оператора. Спецификации описывают поведение программного стека, а не сертификацию безопасности ячейки.

## Карта контрактов

| Раздел | Содержание | Основные источники |
|---|---|---|
| [deployment](specs/deployment/spec.md) | запуск, режимы, конфигурация | `cobot/`, `iiwa_bringup/`, `setting_loader.py` |
| [robot-model](specs/robot-model/spec.md) | модель, суставы, инструмент и лимиты | `iiwa_description/`, `iiwa_config/`, `tool_manager.py` |
| [fri-control](specs/fri-control/spec.md) | связь с Sunrise и аппаратный интерфейс | `iiwa_controller/`, `iiwa_sunrise/` |
| [simulation](specs/simulation/spec.md) | Webots, объекты, камеры | `iiwa_bringup/`, `iiwa_utils/` |
| [motion-planning](specs/motion-planning/spec.md) | суставное, декартово и именованное движение | `iiwa_msgs/`, `move_to_pose_server.py` |
| [motion-arbitration](specs/motion-arbitration/spec.md) | приоритет, TCP-гизма и остановка | `motion_priority.py`, `tcp_gizmo.cpp` |
| [direct-trajectories](specs/direct-trajectories/spec.md) | отправка траекторий JSON/CSV | `trajectory.py`, `direct_trajectory.py` |
| [external-api](specs/external-api/spec.md) | HTTP, MCP и чтение состояния | `iiwa_web/`, `api_endpoints.yaml` |
| [motion-sequences](specs/motion-sequences/spec.md) | последовательности целей и запись | `motion_sequence_runner.py`, `runner.py` |
| [observability](specs/observability/spec.md) | RViz, Foxglove, состояние и диагностика | `iiwa_bringup/`, `iiwa_controller/` |
| [sunrise-program](specs/sunrise-program/spec.md) | контроллерная программа и выбор сети | `iiwa_sunrise/` |
| [project-tooling](specs/project-tooling/spec.md) | CLI, сборка и документация | `cobot/`, `scripts/`, `docker/` |

## Важные различия и открытые вопросы

- `cobot-setting.yaml` служит локальной конфигурацией. Спецификация ссылается на структуру настроек и значения в `src/iiwa_config/config/setting.yaml`, но не закрепляет IP и другие локальные параметры как универсальные.
- `MoveToPose.action` и HTTP `POST /robot/move/pose` принимают KUKA ABC в **радианах**. `MoveToPose.srv` объявляет углы в **градусах** и скорость в процентах; в основном launch эта служба не поднимается. Перед использованием её контракт надо пересмотреть отдельно.
- Веб-сервер и Foxglove по настройкам могут слушать `0.0.0.0`; в просмотренном коде HTTP/MCP не обнаружена прикладная аутентификация. Развёртывание за пределами доверенной сети требует отдельного решения по доступу.
- README обещает WebSocket API; просмотренные исходники `iiwa_web` явно создают HTTP/FastAPI и MCP. Отдельный WebSocket endpoint в них не найден. Foxglove предоставляет собственный WebSocket bridge.
- Именованные позы `home`, `work`, `transport` являются конфигурациями суставов в SRDF. Их наличие не доказывает безопасность конкретной физической сцены, нагрузки или инструмента.
- Пределы ускорения и рывка в `joint_limits.yaml` помечены как проектные, а не подтверждённые характеристики KUKA.
- Прямой API траекторий проверяет суставную кинематику и динамику, но не проверяет столкновения с окружением.
- `ServerFriRos2` перед FRI выполняет подъезд к заданной в Java позиции. Значения `ZERO_POSITION` и `MONITOR_WORKING_POSITION` требуют проверки для каждой физической ячейки.
- При заданном `bag_path` runner удаляет существующий каталог записи перед созданием нового MCAP; путь записи нельзя указывать на данные, которые нужно сохранить.

## Проверка структуры

```bash
openspec list --specs
openspec validate --specs --strict
```
