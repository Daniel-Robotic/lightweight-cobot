# External API

## Purpose

Фиксирует доступные внешним клиентам HTTP и MCP операции для чтения состояния и отправки команд роботу, включая типы данных, проверку ввода и ошибки зависимостей ROS.

## Requirements

### Requirement: Включаемый сервер
При `web.enabled=true` система SHALL запускать FastAPI на настроенных `host` и `port`, предоставлять OpenAPI-описание и монтировать MCP-приложение по `/mcp`.

#### Scenario: Веб-сервер включён
- **WHEN** основной launch загружает `web.enabled=true`
- **THEN** запускается `iiwa_web_server` с путями к конфигурации эндпоинтов и суставных лимитов

### Requirement: Конфигурируемые маршруты
Маршруты из `api_endpoints.yaml` SHALL создавать HTTP обработчики типов `topic`, `service`, `action`, `fk` и `tf` с описанными полями и тайм-аутами.

#### Scenario: Отключённый маршрут
- **WHEN** запись маршрута имеет `enabled=false`
- **THEN** маршрут не появляется в доступном HTTP API

### Requirement: Чтение состояния
`GET /robot/joint_states` SHALL возвращать имена, положения, скорости и усилия из `/joint_states`; `GET /robot/pose` SHALL возвращать TCP относительно `base_link`, XYZ в метрах, кватернион и KUKA ABC в радианах и градусах.

#### Scenario: Нет сообщения суставов
- **WHEN** `/joint_states` не поступает до тайм-аута маршрута
- **THEN** запрос чтения состояния возвращает HTTP 503

### Requirement: Команды движения через HTTP
`POST /robot/move/joints`, `/robot/move/pose` и `/robot/move/named` SHALL передавать проверенные цели в соответствующие ROS action или service и возвращать `success` и `message`.

#### Scenario: Некорректная длина суставов
- **WHEN** тело запроса `/robot/move/joints` содержит не семь углов
- **THEN** запрос получает 422 и ROS action не вызывается

#### Scenario: Нет ROS-сервера
- **WHEN** соответствующий ROS action или service недоступен до тайм-аута
- **THEN** HTTP обработчик возвращает 503

### Requirement: Единицы HTTP-позы
`POST /robot/move/pose` SHALL принимать XYZ в метрах и A/B/C в радианах по соглашению ZYX; `speed` SHALL лежать в `[0.01, 1.0]`.

#### Scenario: Выбор планировщика без учёта регистра
- **WHEN** клиент указывает `planner="PTP"`
- **THEN** маршрут нормализует ключ до `ptp` до отправки action-цели

### Requirement: MCP-представление
MCP-приложение SHALL строиться из тех же FastAPI-маршрутов, которые зарегистрированы для HTTP управления и чтения.

#### Scenario: Запрос каталога инструментов MCP
- **WHEN** MCP-клиент подключается к `/mcp`
- **THEN** операции, выведенные из маршрутов FastAPI, доступны через MCP-приложение

