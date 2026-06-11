import argparse

from cobot import ui

# Import each sub-command's run() so we can call them in sequence.
# Импортируем run() каждой подкоманды, чтобы вызывать их по порядку.
from cobot.commands.doc_setup import run as _doc_setup
from cobot.commands.docker_setup import run as _docker_setup
from cobot.commands.local_setup import run as _local_setup
from cobot.commands.robot_setup import run as _robot_setup


def register(subparsers):
    """Register the "setup" subparser.
    Регистрирует подпарсер "setup".
    """
    p = subparsers.add_parser("setup", help="First-time project setup")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    """Run the three-step first-time setup wizard: docs -> build env -> robot config.
    Запускает трёхшаговый мастер первичной настройки: документация -> среда сборки -> конфиг.
    """
    ui.header("Первичная настройка", "3 шага")

    # Step 1 - documentation server.
    # Шаг 1 - сервер документации.
    if ui.confirm("Шаг 1/3 — настроить сервер документации?", default=True):
        _doc_setup(args)

    # Step 2 - build environment: local ROS2 or Docker.
    # Шаг 2 - среда сборки: локальный ROS2 или Docker.
    env_choice = ui.select(
        "Шаг 2/3 — как настроить среду сборки?",
        [
            "local-setup  — установить ROS2 Jazzy на эту машину и собрать через colcon",
            "docker-setup — собрать Docker-образ с предустановленным ROS2 Jazzy",
        ],
        "local-setup  — установить ROS2 Jazzy на эту машину и собрать через colcon",
    )
    if env_choice is None:
        return
    if env_choice.startswith("local"):
        _local_setup(args)
    else:
        _docker_setup(args)

    # Step 3 - robot parameters in cobot-setting.yaml.
    # Шаг 3 - параметры робота в cobot-setting.yaml.
    if ui.confirm("Шаг 3/3 — настроить параметры робота (cobot-setting.yaml)?", default=True):
        _robot_setup(args)
