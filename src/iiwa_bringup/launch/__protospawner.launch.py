# TODO: код на будущее

import os
import re
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.urdf_spawner import URDFSpawner


def _patch_proto_controller(proto_path: str):
    pattern = r'(field\s+SFString\s+controller\s+)"void"'
    replacement = r'\1"exec"'
    proto = Path(proto_path)
    text = proto.read_text(encoding='utf-8')
    new_text, count = re.subn(pattern, replacement, text)

    if count != 0:
        proto.write_text(new_text, encoding='utf-8')


def _after_xacro(context, *args, **kwargs):
    """Выполняется после завершения xacro2proto: патчим proto и возвращаем actions (webots, rsp)."""
    proto_path = kwargs['proto_path']
    # проверим существование
    if not Path(proto_path).exists():
        raise RuntimeError(f"Expected proto file not found: {proto_path}")

    # Патчим
    _patch_proto_controller(proto_path)

    # создаём действия, которые будут запущены ПОСЛЕ xacro2proto
    world_path = LaunchConfiguration('world').perform(context)
    webots = WebotsLauncher(world=world_path, 
                            ros2_supervisor=True)

    # robot_state_publisher можно запускать после webots (или вместе)
    robot_description_sub: Command = kwargs['robot_description_sub']
    robot_description_str = robot_description_sub.perform(context)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_str,
            "use_sim_time": True
        }]
    )

    # вернуть список действий, которые launch затем подключит
    return [ webots, rsp_node ]

def _runtime_setup(context, *args, **kwargs):
    # Получаем значения
    model_path = LaunchConfiguration('model').perform(context)
    protos_path = LaunchConfiguration('protos_path').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    tool_slot = LaunchConfiguration('tool_slot').perform(context)
    transform_proto = LaunchConfiguration('transform_proto').perform(context)
    rotation_proto = LaunchConfiguration('rotation_proto').perform(context)



    # убедимся, что директория существует
    os.makedirs(protos_path, exist_ok=True)

    proto_output = f"{protos_path}/{robot_name}.proto"   # <- явное .proto

    xacro2proto_node = Node(
        package="webots_ros2_importer",
        executable="xacro2proto",
        name="xacro2proto",
        output="screen",
        arguments=[
            "--input", str(model_path),
            "--output", proto_output,
            "--tool-slot", str(tool_slot),
            "--translation", str(transform_proto).replace(",", " "),
            "--rotation", str(rotation_proto).replace(",", " "),
        ]
    )

    # обработчик: после завершения xacro2proto запускаем OpaqueFunction, который патчит и запускает webots+rsp
    after_xacro = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2proto_node,
            on_exit=[
                OpaqueFunction(
                    function=_after_xacro,
                    kwargs={
                        'proto_path': proto_output,
                        'robot_description_sub': kwargs['robot_description_sub']
                    }
                )
            ]
        )
    )

    return [ xacro2proto_node, after_xacro ]

def generate_launch_description():
    description_pkg = "iiwa_description"

    declare_model_arg = DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution([
            FindPackageShare(description_pkg),
            "urdf",
            "iiwa7.urdf.xacro"
        ]),
    )

    declare_robot_name_arg = DeclareLaunchArgument(name="robot_name", default_value="iiwa7")
    declare_world_arg = DeclareLaunchArgument(
        name="world",
        default_value=PathJoinSubstitution([
            FindPackageShare(description_pkg),
            "worlds",
            "simple_world.wbt"
        ]),
    )
    declare_proto_arg = DeclareLaunchArgument(
        name="protos_path",
        default_value=PathJoinSubstitution([
            FindPackageShare(description_pkg),
            "protos"
        ]),
    )
    declare_transform_arg = DeclareLaunchArgument(
        name="transform_proto",
        default_value="0 0 0"
    )
    declare_rotation_arg = DeclareLaunchArgument(
        name="rotation_proto",
        default_value="0 0 1 0"
    )
    declare_tool_arg = DeclareLaunchArgument(name="tool_slot", default_value="link7_ee")


    robot_description_cmd = Command([
        "xacro ", LaunchConfiguration("model"),
        " robot_name:=", LaunchConfiguration("robot_name")
    ])

    runtime_setup = OpaqueFunction(
        function=_runtime_setup,
        kwargs={'robot_description_sub': robot_description_cmd}
    )

    ld = LaunchDescription([
        declare_model_arg,
        declare_robot_name_arg,
        declare_world_arg,
        declare_proto_arg,
        declare_transform_arg,
        declare_rotation_arg,
        declare_tool_arg,
        runtime_setup,
    ])
    return ld
