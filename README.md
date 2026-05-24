# Lightweight Cobot

> **Тестовая версия документации — будет дополнена**

ROS 2 пакеты для управления роботом **KUKA LBR IIWA 7 R800**: связь с реальным роботом через библиотеку [FRI](https://github.com/lbr-stack/fri) (Fast Robot Interface) и симуляция в среде [Webots](https://cyberbotics.com/).

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

---

## Статус

| ОС | Дистрибутив ROS | Версия FRI |
| :--- | :--- | :--- |
| `Ubuntu 24.04` | `jazzy` | `1.15` |

---

## Быстрый старт

Для установки запустите скрипт одной командой:

```bash
curl -fsSL https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/main/install.sh
```

Скрипт установит ROS 2 Jazzy, Webots, соберёт рабочее пространство и установит CLI `cobot`.

---

## Команды `cobot`

После установки управляйте системой через CLI:

```
cobot <команда>
```

### Настройка

| Команда | Описание |
| :--- | :--- |
| `cobot setup` | Первоначальная настройка: документация, среда сборки, конфигурация робота |
| `cobot local-setup` | Установка ROS 2 Jazzy локально и сборка проекта через colcon |
| `cobot docker-setup` | Сборка или загрузка Docker-образов |
| `cobot doc-setup` | Запуск или остановка сервера документации MkDocs |
| `cobot robot-setup` | Интерактивная настройка файла `cobot-setting.yaml` |

### Запуск

| Команда | Описание |
| :--- | :--- |
| `cobot run` | Запуск контроллера робота или симулятора Webots (локально или через Docker) |

### Сборка

| Команда | Описание |
| :--- | :--- |
| `cobot rebuild` | Пересборка ROS 2 пакетов из `src/` с помощью colcon |
| `cobot clean` | Удаление артефактов сборки (`build/` `install/` `log/`) |

### Управление

| Команда | Описание |
| :--- | :--- |
| `cobot update` | Получение последних изменений из удалённой ветки и переустановка `cobot` |
| `cobot delete` | Удаление проекта, Docker-образов, контейнеров и опционально ROS 2 |

---

## Демонстрация

> GIF-анимации будут добавлены в следующих версиях

<table>
  <tr>
    <th align="center" width="33%">Симуляция в Webots</th>
    <th align="center" width="33%">Управление по суставам</th>
    <th align="center" width="33%">Декартово управление</th>
  </tr>
  <tr>
    <td align="center"><i>— скоро —</i></td>
    <td align="center"><i>— скоро —</i></td>
    <td align="center"><i>— скоро —</i></td>
  </tr>
</table>

---

## Пакеты

| Пакет | Описание |
| :--- | :--- |
| `iiwa_bringup` | Launch-файлы: симуляция Webots, реальный робот через FRI, MoveIt и RViz |
| `iiwa_config` | Конфигурационные файлы: MoveIt, контроллеры ros2_control, кинематика и общие параметры |
| `iiwa_controller` | Hardware interface: управление суставами в реальном времени через FRI |
| `iiwa_description` | URDF/XACRO описание робота и конфигурация мира Webots |
| `iiwa_msgs` | ROS 2 интерфейсы: action-сообщения для движения по суставам и в декартовых координатах, сервисы именованных поз |
| `iiwa_planning` | Планирование движения: C++ и Python узлы на базе MoveIt 2 (OMPL, Pilz, moveit_py) |
| `iiwa_utils` | Утилиты системы: загрузка конфигурации, спавн объектов и камер в Webots, конвертация данных |
| `iiwa_web` | Веб-интерфейс для мониторинга и дистанционного управления через браузер |

---

## Цитирование

Если вы используете этот проект в своей работе, пожалуйста, оставьте звёздочку ⭐ и укажите ссылку:

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

Выражаем благодарность следующим организациям и грантам:

| Организация | Примечание |
| :--- | :--- |
| [Комсомольский-на-Амуре государственный университет](https://knastu.ru/) | Исследования проводились на базе КнАГУ |
| [Российский научный фонд](https://rscf.ru/) | Работа выполнена при поддержке Российского научного фонда |
| <!-- TODO --> | <!-- TODO --> |
| <!-- TODO --> | <!-- TODO --> |

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
# Просто выполнить последовательность без записи
ros2 run iiwa_utils test_motion_sequence \
  --ros-args -p n_iterations:=3 \
             -p delay_between_iterations:=5.0

# Записать все доступные топики в bag
ros2 run iiwa_utils test_motion_sequence \
  --ros-args -p n_iterations:=5 \
             -p delay_between_iterations:=5.0 \
             -p bag_path:=/tmp/iiwa_session

# Записать конкретные топики
ros2 run iiwa_utils test_motion_sequence \
  --ros-args -p n_iterations:=5 \
             -p delay_between_iterations:=5.0 \
             -p bag_path:=/tmp/iiwa_session \
             -p topics:="['/joint_states', '/d455_top/color/image_raw', '/tf']"

# Использовать свой конфиг поз
ros2 run iiwa_utils test_motion_sequence \
  --ros-args -p config_path:=/path/to/my_config.json \
             -p n_iterations:=1 \
             -p delay_between_iterations:=3.0 \
             -p bag_path:=/tmp/iiwa_session

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
