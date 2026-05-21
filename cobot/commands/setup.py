import argparse
from typing import List, Optional

from textual.app import App

# Import each sub-command's run() so we can call them in sequence.
# Импортируем run() каждой подкоманды, чтобы вызывать их по порядку.
from cobot.commands.doc_setup import run as _doc_setup
from cobot.commands.docker_setup import run as _docker_setup
from cobot.commands.local_setup import run as _local_setup
from cobot.commands.robot_setup import run as _robot_setup
from cobot.tui import SCREEN_CSS, PickScreen


# A minimal Textual app that asks a single question and exits with the chosen value.
# We need this because Textual screens cannot run outside of an App context.
# Минимальное Textual-приложение, которое задаёт один вопрос и выходит с выбранным значением.
# Нам это нужно, потому что экраны Textual не могут работать вне контекста приложения.
class _Ask(App[Optional[str]]):
    CSS = SCREEN_CSS

    def __init__(self, step: str, question: str, options: List[str], default: str):
        super().__init__()
        self._step = step
        self._question = question
        self._options = options
        self._default = default

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(self._step, self._question, self._options, self._default),
            self.exit,
        )


def _ask(step: str, question: str, options: List[str], default: str) -> Optional[str]:
    # Returns None if the user pressed Escape to cancel the whole wizard.
    # Возвращает None если пользователь нажал Escape для отмены всего мастера.
    return _Ask(step, question, options, default).run()


def register(subparsers):
    p = subparsers.add_parser("setup", help="First-time project setup")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    # Step 1 - documentation server.
    # Шаг 1 - сервер документации.
    v = _ask("Step 1 of 3", "Set up the documentation server?", ["Yes", "No"], "Yes")
    if v is None:
        return
    if v == "Yes":
        _doc_setup(args)

    # Step 2 - build environment: local ROS2 or Docker.
    # Шаг 2 - среда сборки: локальный ROS2 или Docker.
    v = _ask(
        "Step 2 of 3",
        "How do you want to set up the build environment?",
        [
            "local-setup  — install ROS2 Jazzy on this machine and build with colcon",
            "docker-setup — build a Docker image with ROS2 Jazzy pre-installed",
        ],
        "local-setup  — install ROS2 Jazzy on this machine and build with colcon",
    )
    if v is None:
        return
    if v.startswith("local"):
        _local_setup(args)
    else:
        _docker_setup(args)

    # Step 3 - robot parameters in cobot-setting.yaml.
    # Шаг 3 - параметры робота в cobot-setting.yaml.
    v = _ask(
        "Step 3 of 3",
        "Configure robot parameters (cobot-setting.yaml)?",
        ["Yes", "No"],
        "Yes",
    )
    if v is None:
        return
    if v == "Yes":
        _robot_setup(args)
