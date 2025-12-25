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
                    ros-${ROS_DISTRO}-moveit-py

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

Спавн объекта:
```bash
{
    "data": "Solid { name \"test_box2\" translation 0 1 0.5 children [ Shape { appearance PBRAppearance { baseColor 0.901961 0.380392 0 } geometry Box { size 0.1 0.1 0.1 } } ] boundingObject Box { size 0.1 0.1 0.1 } physics Physics { } }"
}
```

Спавн `.proto`:


Можно заготовить готовые `.proto` файлы, и потом случайно спавнить объект по такому принципу + создать Node который будет вызываться и спавнить этот объекты. Может быть шаблон куда потом подставятся данные через `.format()`. По такому же принципу спавн человека. Остается понять, только задать область спавна относительно робта