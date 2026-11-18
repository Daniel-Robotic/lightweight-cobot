# Repository Guidelines

## Рабочий процесс агентов и SDD

- Пользователь формулирует задачу в Codex. До изменения кода Codex изучает текущую реализацию и создаёт или обновляет спецификацию OpenSpec на русском языке в `openspec/changes/`; требования должны описывать проверяемое поведение, ограничения, отказные сценарии и критерии приёмки. Для небольшого исправления допустима краткая спецификация, но не пропуск этапа.
- Реализацию делегируй через **видимый дочерний BB-тред OpenCode**, чтобы пользователь видел прогресс и результаты в интерфейсе BB. Используй provider `acp-opencode` и модель `opencode-go/deepseek-v4.1-flash` (отображаемое имя: **DeepSeek V4.1 Flash 1017**). Не запускай OpenCode как скрытый локальный CLI-процесс и не подменяй provider или модель. Если этот маршрут недоступен, сообщи о блокере; не переходи на Polza или другую модель без прямого указания пользователя.
- Перед созданием треда проверь `bb status` и доступность модели через `bb provider models acp-opencode --environment <текущая-среда>`. Создавай задачу командой `bb thread spawn` с `--parent-self`, текущими `--project` и `--environment`, `--provider acp-opencode`, `--model opencode-go/deepseek-v4.1-flash` и `--visibility visible`. Передавай исполнителю ссылку на change OpenSpec, границы задачи и команды проверки; для подробного задания используй `--prompt-file`. После реализации Codex сверяет diff с требованиями, актуализирует основную спецификацию в `openspec/specs/` и закрывает пункты задач только при наличии фактического подтверждения.
- При каждом изменении поведения, установки, конфигурации или процесса разработки проверяй и обновляй корневые `README.md` и `README_en.md` согласно навыку `readme-authoring`. Не добавляй в README неподтверждённые команды и версии.
- Пользовательская и операторская документация ведётся в `doc/lwc-doc/docs/` как MkDocs Material с русской и английской страницами (`*.md` и `*.en.md`). Изменения, влияющие на настройку, запуск, интерфейсы или рабочий процесс, должны отражаться на обеих языковых страницах и в `doc/lwc-doc/mkdocs.yml` при необходимости. Сначала проверяй существующую инфраструктуру и дополняй её, не создавая параллельный сайт.
- Навыки проекта размещаются в `.agents/skills/` для совместного обнаружения Codex, OpenCode и BB. Используй релевантный установленный навык по его описанию; OpenSpec-артефакты и спецификации проекта пиши по-русски.
- Не запускай команды физического робота как часть автоматической проверки. Для изменений движения сначала планируй симуляцию Webots; проверку на физическом роботе выполняет оператор с соблюдением аппаратных мер безопасности.

## Project Structure & Module Organization

Lightweight Cobot targets Ubuntu 24.04, ROS 2 Jazzy, and the KUKA LBR iiwa 7 R800.

- `src/iiwa_controller/`: C++ FRI hardware interface; bundled SDK in `external/libFRI/`.
- `src/iiwa_planning/` and `src/iiwa_msgs/`: motion planning nodes and ROS actions/services.
- `src/iiwa_bringup/`, `src/iiwa_config/`, and `src/iiwa_description/`: launch files, configuration, URDF/Xacro, meshes, and Webots worlds.
- `src/iiwa_utils/` and `src/iiwa_web/`: Python utilities and REST/WebSocket/MCP interfaces.
- `src/iiwa_sunrise/`: controller-side Java applications, built separately in Sunrise Workbench.
- `cobot/`: Python CLI; `scripts/` and `docker/`: environment setup.
- `doc/lwc-doc/docs/`: bilingual documentation (`*.md` and `*.en.md`).

## Build, Test, and Development Commands

Run commands from the repository root after installing prerequisites with `cobot setup`.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
colcon test --base-paths src
colcon test-result --verbose
```

These commands build the workspace, activate its packages, and run/report registered tests. Specify `--base-paths src` because the repository root contains `COLCON_IGNORE`.

- `cobot rebuild iiwa_controller --symlink-install`: rebuild one package; build dependencies first.
- `cobot run local`: launch interactively; select **Webots simulator** for development.
- `cobot doc-setup`: serve documentation locally using Docker.
- `cobot doc-setup build`: generate the static documentation and PDF.

## Coding Style & Naming Conventions

Use four-space indentation for Python and follow adjacent code for C++ formatting (the hardware interface uses two spaces). C++ targets C++17. Use `snake_case` for Python modules/functions, `PascalCase` for classes and ROS interface files, and `*.launch.py` for launch files. Preserve existing C++ API names. No repository-wide formatter configuration is present; avoid unrelated formatting and bundled SDK edits.

## Testing Guidelines

Manifests declare pytest for Python packages and ament lint dependencies for several CMake packages, but no dedicated test suite or coverage threshold is established. Place new tests under the relevant package's `test/` directory, using `test_*.py` for pytest, and register them with its build system. Record Webots smoke-test steps for motion or launch changes; distinguish simulation validation from physical-hardware validation.

## Commit & Pull Request Guidelines

History mixes imperative summaries with `feat:`, `fix:`, `docs:`, and `refactor:` prefixes; prefer a concise prefixed summary. Keep commits focused. PRs should explain the behavior change, affected packages, validation commands/results, and related issues. Include screenshots for visual changes and update both documentation languages when applicable.

## Configuration & Hardware Safety

Review `cobot-setting.yaml` before launching. Validate motion changes in simulation first. Physical runs require checking robot networking, joint limits, active tool/load data, control mode, and a clear work area with operator supervision.
