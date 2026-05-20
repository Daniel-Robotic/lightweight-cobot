import argparse
from typing import Optional

from textual.app import App

from cobot.commands.doc_setup import run as _doc_setup
from cobot.commands.docker_setup import run as _docker_setup
from cobot.commands.local_setup import run as _local_setup
from cobot.commands.robot_setup import run as _robot_setup
from cobot.tui import SCREEN_CSS, PickScreen


class _AskSetupMode(App[Optional[str]]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(
                "Step 1 of 1",
                "How do you want to set up the project?",
                [
                    "local-setup  — install ROS2 Jazzy on this machine and build with colcon",
                    "docker-setup — build a Docker image with ROS2 Jazzy pre-installed",
                ],
                "local-setup  — install ROS2 Jazzy on this machine and build with colcon",
            ),
            self.exit,
        )


def register(subparsers):
    p = subparsers.add_parser("setup", help="First-time project setup")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    
    _doc_setup(args)
    
    choice = _AskSetupMode().run()
    if choice is None:
        return

    if choice.startswith("local"):
        _local_setup(args)
    else:
        _docker_setup(args)

    _robot_setup(args)
