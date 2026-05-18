## Описание пакетов

|Наименование|описание|
|---|---|
|iiwa_bringup|Все необходимые файлы запуска расположены в этом пакете|
|iiwa_config|Файлы конфигурации и основной `setting.yam` файл расположены внутри этого пакета|
|iiwa_controller|Самописный контроллер на физического робота|
|iiwa_description|urdf/xacro файлы, а также все 3D объекты и webots миры находятся в этом пакете|
|iiwa_utils|Вспомогательные модули или функции для рабоы всей системы|



> Вроде уже не обязательно
Подмена файла по пути обязательна: `/opt/ros/rolling/lib/webots_ros2_driver/ros2_supervisor.py`
необходимо `warn` заменить на `warning` в логере

```bash

sudo apt install -y ros-${ROS_DISTRO}-webots-ros2 \
                    ros-${ROS_DISTRO}-ros2-control \ 
                    ros-${ROS_DISTRO}-ros2-controllers \
                    ros-${ROS_DISTRO}-moveit \
                    ros-${ROS_DISTRO}-moveit-py \
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