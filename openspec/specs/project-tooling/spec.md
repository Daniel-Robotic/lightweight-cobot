# Project Tooling

## Purpose

Описывает операторские команды для настройки, сборки, запуска и локального просмотра документации проекта без прямого вмешательства в циклическое управление.

## Requirements

### Requirement: Мастер первичной настройки
`cobot setup` SHALL последовательно предлагать настройку документации, параметров робота и среды сборки.

#### Scenario: Первый запуск
- **WHEN** пользователь вызывает `cobot setup`
- **THEN** получает отдельный выбор для локальной документации, `cobot-setting.yaml` и среды ROS 2 или Docker

### Requirement: Редактирование конфигурации
`cobot robot-setup` SHALL сохранять редактируемые значения в `cobot-setting.yaml` и применять выбранный из реестра инструмент к модели робота.

#### Scenario: Выбор другого инструмента
- **WHEN** пользователь меняет `tool.active`
- **THEN** конфигурация и генерируемые Xacro/SRDF файлы соответствуют выбранному инструменту

### Requirement: Сборка из src
`cobot rebuild` SHALL запускать `colcon build --base-paths src` для всех пакетов или выбранного подмножества и поддерживать `--symlink-install`.

#### Scenario: Пересборка одного пакета
- **WHEN** пользователь задаёт `cobot rebuild iiwa_controller --symlink-install`
- **THEN** CLI передаёт пакет и флаг символьной установки в colcon

### Requirement: Локальная документация
`cobot doc-setup` SHALL предоставлять запуск документации, сборку статического сайта и PDF, пересборку и остановку сервера.

#### Scenario: Сборка документации
- **WHEN** пользователь вызывает `cobot doc-setup build`
- **THEN** создаются статическая документация и PDF из исходников `doc/lwc-doc/`

### Requirement: Разделение файлов Sunrise и colcon
ROS workspace MUST исключать `src/iiwa_sunrise` из colcon-сборки, оставляя Java-исходники для отдельной сборки в Sunrise Workbench.

#### Scenario: Сборка рабочего пространства
- **WHEN** выполняется `colcon build --base-paths src`
- **THEN** пакет контроллерных Java-программ не собирается colcon

