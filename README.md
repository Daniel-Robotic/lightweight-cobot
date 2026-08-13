# Lightweight Cobot

<p align="center">
  <strong>Русский</strong> · <a href="README_en.md">English</a>
</p>

**Lightweight Cobot (LWC)** — открытая система управления коллаборативным роботом **KUKA LBR iiwa 7 R800** на базе ROS 2. Проект объединяет работу с физическим роботом через FRI и `ros2_control`, цифровой двойник в Webots, планирование движений MoveIt 2, визуализацию в RViz и Foxglove, а также REST API и MCP для внешних приложений и AI-агентов.

<table>
  <tr>
    <th align="center">LBR IIWA 7 R800</th>
  </tr>
  <tr>
    <td align="center">
      <img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/jazzy/lbr_fri_ros2_stack/doc/img/foxglove/iiwa7_r800.png" alt="LBR IIWA 7 R800" width="300">
    </td>
  </tr>
</table>

## Какие задачи решает проект

- Даёт единый программный стек для физического робота и симуляции без дублирования управляющего кода.
- Подключает KUKA Sunrise Cabinet к ROS 2 через FRI и предоставляет стандартные интерфейсы `ros2_control`.
- Выполняет суставные и декартовы движения с помощью MoveIt 2, OMPL и Pilz.
- Упрощает установку, настройку, сборку и запуск через CLI `cobot`.
- Хранит основные параметры робота, инструментов и сервисов в одном файле `cobot-setting.yaml`.
- Предоставляет средства мониторинга и интеграции через RViz, Foxglove, HTTP/WebSocket API и MCP.

## Возможности

| Компонент | Назначение |
|---|---|
| Физический робот | Управление KUKA LBR iiwa 7 R800 через FRI и `ServerFriRos2` |
| Цифровой двойник | Симуляция робота, инструментов и окружения в Webots |
| Планирование | Суставные и декартовы траектории через MoveIt 2 |
| Управление | `ros2_control`, ROS 2 actions/services, REST API и MCP |
| Наблюдение | RViz, Foxglove и состояние системы через веб-интерфейс |
| Инфраструктура | Локальное окружение или Docker, единый CLI и централизованная конфигурация |

## Совместимость

| Компонент | Поддерживаемая версия |
|---|---|
| Операционная система | **Ubuntu 24.04 LTS** — подтверждённая ОС для нативной установки |
| ROS 2 | Jazzy |
| Webots | 2025a |
| Python для CLI | 3.11 |
| KUKA Sunrise OS | 1.16 |
| KUKA FRI | 1.16 |

Docker можно использовать как альтернативную среду на совместимом Linux-хосте. Полноценная работа проекта на Windows и macOS не заявлена. Sunrise Workbench используется отдельно для подготовки и синхронизации проекта контроллера KUKA.

## Репозитории и документация

| Ресурс | Ссылка |
|---|---|
| Основной репозиторий | [GitVerse](https://gitverse.ru/daniel-robotics/lightweight-cobot) |
| Зеркало | [GitHub](https://github.com/Daniel-Robotic/lightweight-cobot) |
| Онлайн-документация | [GitVerse Pages](https://daniel-robotics.gitverse.site/lightweight-cobot/) |
| Зеркало документации | [GitHub Pages](https://daniel-robotic.github.io/lightweight-cobot/) |

Подробные инструкции начинаются со страницы [«Обзор»](doc/lwc-doc/docs/getting-started/index.ru.md). Исходные тексты документации находятся в `doc/lwc-doc/docs`.

## Быстрый старт

### Требования

- Ubuntu 24.04 LTS;
- доступ в интернет;
- права `sudo`;
- физический KUKA LBR iiwa 7 R800 либо компьютер для работы только с симулятором.

### Установка CLI

Запустите установщик:

```bash
curl -fsSL https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/master/install.sh | bash
```

Установщик проверит базовые инструменты, установит Docker, `uv` и Python 3.11 при необходимости, склонирует проект в `~/.lwc` и установит CLI `cobot`. Каталог можно изменить переменной `COBOT_INSTALL_DIR`.

Откройте новый терминал или обновите окружение оболочки, затем запустите мастер первоначальной настройки:

```bash
cobot setup
```

Мастер последовательно предложит запустить локальную документацию, настроить `cobot-setting.yaml` и выбрать среду сборки: нативный ROS 2 или Docker.

### Только симуляция

Для работы в Webots физический робот и Sunrise Workbench не требуются. Выполните:

```bash
cobot run
```

Выберите локальную или Docker-среду, а затем пункт **Симулятор Webots**.

### Физический робот

Перед первым запуском подготовьте контроллер и программу `ServerFriRos2` по инструкции [«Настройка SunriseWorkbench»](doc/lwc-doc/docs/getting-started/sunrise-setup.ru.md). Проверьте сеть KONI/KLI, IP-адреса, период FRI, выбранный инструмент и его Load Data.

После настройки запустите:

```bash
cobot run
```

Выберите локальную или Docker-среду, а затем пункт **Физический контроллер**. Подробное описание серверной программы приведено на странице [ServerFriRos2](doc/lwc-doc/docs/sunrise/kuka/programs/server-fri-ros2.ru.md).

## Основные команды

| Команда | Назначение |
|---|---|
| `cobot setup` | Первоначальная настройка документации, робота и среды сборки |
| `cobot robot-setup` | Интерактивное изменение `cobot-setting.yaml` |
| `cobot local-setup` | Установка ROS 2 Jazzy и локальная сборка workspace |
| `cobot docker-setup` | Загрузка или сборка Docker-образов |
| `cobot run` | Интерактивный выбор среды и запуск робота или Webots |
| `cobot run local` | Запуск через нативный ROS 2 с последующим выбором робота или Webots |
| `cobot run docker` | Запуск в Docker с последующим выбором робота или Webots |
| `cobot rebuild` | Пересборка ROS 2 workspace |
| `cobot clean` | Удаление артефактов `build`, `install` и `log` |
| `cobot update` | Обновление проекта и переустановка CLI |
| `cobot --help` | Полный список доступных команд |

## Документация локально

Для локального просмотра необходим Docker.

```bash
cobot doc-setup
```

По умолчанию сайт будет доступен по адресу [http://localhost:8000](http://localhost:8000). Исходные Markdown-файлы отслеживаются автоматически.

| Команда | Назначение |
|---|---|
| `cobot doc-setup` | Запустить локальный сервер документации |
| `cobot doc-setup build` | Собрать статический сайт и единый PDF в `doc/lwc-doc/site` |
| `cobot doc-setup rebuild` | Пересобрать Docker-образ и перезапустить сервер |
| `cobot doc-setup down` | Остановить локальный сервер |

Онлайн-версия доступна на [GitVerse Pages](https://daniel-robotics.gitverse.site/lightweight-cobot/), зеркало — на [GitHub Pages](https://daniel-robotic.github.io/lightweight-cobot/).

## Пакеты

| Пакет | Описание |
|---|---|
| `iiwa_bringup` | Launch-файлы: симуляция Webots, реальный робот через FRI, MoveIt и RViz |
| `iiwa_config` | Конфигурационные файлы: MoveIt, контроллеры ros2_control, кинематика и общие параметры |
| `iiwa_controller` | Hardware interface: управление суставами в реальном времени через FRI |
| `iiwa_description` | URDF/XACRO описание робота и конфигурация мира Webots |
| `iiwa_msgs` | ROS 2 интерфейсы: action-сообщения для движения по суставам и в декартовых координатах, сервисы именованных поз |
| `iiwa_planning` | Планирование движения: C++ и Python узлы на базе MoveIt 2 (OMPL, Pilz, moveit_py) |
| `iiwa_utils` | Утилиты системы: загрузка конфигурации, спавн объектов и камер в Webots, конвертация данных |
| `iiwa_web` | REST API, WebSocket и MCP для мониторинга и внешнего управления |

Java-программы для KUKA Sunrise Cabinet находятся отдельно в `src/iiwa_sunrise` и не входят в сборку colcon.

## Безопасность

Перед отправкой команд на физический робот проверьте рабочую область, ограничения суставов, активный инструмент, модель нагрузки и выбранный режим управления. LWC не заменяет штатные средства безопасности KUKA, оценку рисков роботизированной ячейки и контроль оператора.

## Лицензия

Проект распространяется по лицензии [Apache License 2.0](LICENSE).

## Цитирование

Если вы используете проект в исследовании или разработке, укажите ссылку на репозиторий:

```bibtex
@software{lightweight_cobot_2026,
  author  = {Грабарь, Даниил},
  title   = {Lightweight Cobot: ROS 2 stack for KUKA LBR IIWA 7},
  year    = {2026},
  url     = {https://gitverse.ru/daniel-robotics/lightweight-cobot}
}
```

---

## Благодарности

| Организация | Примечание |
|---|---|
| [Комсомольский-на-Амуре государственный университет](https://knastu.ru/) | Исследования проводились на базе КнАГУ |
| [Российский научный фонд](https://rscf.ru/) | Работа выполнена при поддержке Российского научного фонда |

---

<!-- ============================================================ -->
<!-- Черновые команды — будут удалены в следующих версиях         -->
<!-- ============================================================ -->

## Черновые команды

> Вроде уже не обязательно
Подмена файла по пути обязательна: `/opt/ros/rolling/lib/webots_ros2_driver/ros2_supervisor.py`
необходимо `warn` заменить на `warning` в логере

```bash

sudo apt install -y ros-${ROS_DISTRO}-webots-ros2 \
                    ros-${ROS_DISTRO}-ros2-control \ 
                    ros-${ROS_DISTRO}-ros2-controllers \
                    ros-${ROS_DISTRO}-moveit \
                    ros-${ROS_DISTRO}-moveit-py \
                    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
                    ros-${ROS_DISTRO}-ament-cmake-clang-format \
                    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
                    ros-${ROS_DISTRO}-librealsense2 \
                    ros-${ROS_DISTRO}-realsense2* \
                    libportaudio2 \

```

Установка moveit2 (внимательно проверяй)
```bash
sudo apt install -y build-essential \
                    cmake \
                    git \
                    python3-colcon-common-extensions \
                    python3-flake8 \
                    python3-rosdep \
                    python3-setuptools \
                    python3-vcstool \
                    wget

# Not Using
LC_ALL=C ros2 launch iiwa_moveit demo...

sudo pip3 install transforms3d --break-system-packages
```
Отправка робота в точку:
```bash
ros2 action send_goal /iiwa/move_to_pose iiwa_msgs/action/MoveToPose \
  "{x: 0.5, y: 0.0, z: 0.5, a: 3.14, b: 0, c: 0, speed: 0.1, planner: 'ptp'}"

ros2 action send_goal --feedback /iiwa/move_to_joints iiwa_msgs/action/MoveToJoints \
  "{joints: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0], speed: 0.4}"

ros2 service call /iiwa/move_to_named iiwa_msgs/srv/MoveToNamedPose \
  "{name: 'home', speed: 0.5}"

ros2 service call /iiwa/move_to_named iiwa_msgs/srv/MoveToNamedPose \
  "{name: 'work', speed: 0.3}"

ros2 service call /iiwa/stop std_srvs/srv/Trigger "{}"
```

Примеры использования `test_motion_sequence`:
```bash
ros2 run iiwa_planning motion_sequence_runner \
  --ros-args \
  -p config_path:=/path/to/config.json \
  -p n_iterations:=3 \
  -p delay_between_iterations:=5.0 \
  -p bag_path:=/tmp/my_bag \
  -p joints_action:=my_ns/move_to_joints \
  -p pose_action:=my_ns/move_to_pose

```

Спавн объекта:
```bash
{
    "data": "Solid { name \"test_box2\" translation 0 1 0.5 children [ Shape { appearance PBRAppearance { baseColor 0.901961 0.380392 0 } geometry Box { size 0.1 0.1 0.1 } } ] boundingObject Box { size 0.1 0.1 0.1 } physics Physics { } }"
}
```

Спавн `.proto`:


Можно заготовить готовые `.proto` файлы, и потом случайно спавнить объект по такому принципу + создать Node который будет вызываться и спавнить этот объекты. Может быть шаблон куда потом подставятся данные через `.format()`. По такому же принципу спавн человека. Остается понять, только задать область спавна относительно робта

Установка зависимостей для сборки пакетов:

```bash
rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:jammy
```

Запись данных в rosbag:

```bash
$ sudo apt-get install ros-$ROS_DISTRO-rosbag2-storage-mcap

# Record all topics
ros2 bag record -a --storage mcap -o my_session

# Record specific topics
ros2 bag record \
  /camera/image_raw \
  /lidar/points \
  /imu/data \
  --storage mcap \
  -o robot_drive_session


# LZ4 = faster write, moderate compression (good for real-time recording)
ros2 bag record -a --storage mcap \
  --compression-mode file \
  --compression-format lz4 \
  -o compressed_session

# Zstandard = slower write, better compression (good for post-processing)
ros2 bag record -a --storage mcap \
  --compression-mode file \
  --compression-format zstd \
  -o compressed_zstd_session

# convert existing rosbag2 to mcap format
ros2 bag convert \
  -i robot_drive_old/ \
  -o robot_drive_mcap/ \
  --output-options '{"storage_id": "mcap"}'
```


<!-- Generate Doc -->
```bash
chmod +x doc/build_docs.sh

./build_docs.sh #Собирает статику в mkdocs/site/
./build_docs.sh build # То же самое явно
./build_docs.sh serve # Live-preview с авто-перезагрузкой
```

<!-- iiwa controller - docker -->
<!-- Lightweight cobot -->
```bash
docker build -t evilfisru/lwc:... -f docker/jazzy/.../Dockerfile .

docker run -it --rm --network host evilfisru/lwa:jazzy-lwa7-noble
```

```bash
# release - src/build/log удаляются (по умолчанию)
`docker build -t my-image .`

# dev - src остаётся для отладки
`docker build --build-arg BUILD_TYPE=dev -t my-image .`
```

```
# URL для доступа к MCP LLM
http://localhost:8007/mcp/mcp

```
