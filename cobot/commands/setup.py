import argparse
from typing import List, Optional

from textual.app import App

from cobot.commands.doc_setup import run as _doc_setup
from cobot.commands.docker_setup import run as _docker_setup
from cobot.commands.local_setup import run as _local_setup
from cobot.commands.robot_setup import run as _robot_setup
from cobot.tui import SCREEN_CSS, PickScreen


class _Ask(App[Optional[str]]):
    """Single-question picker that exits immediately with the chosen value."""
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
    return _Ask(step, question, options, default).run()


def register(subparsers):
    p = subparsers.add_parser("setup", help="First-time project setup")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    # Step 1 — documentation
    v = _ask("Step 1 of 3", "Set up the documentation server?", ["Yes", "No"], "Yes")
    if v is None:
        return
    if v == "Yes":
        _doc_setup(args)

    # Step 2 — build environment
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

    # Step 3 — robot parameters
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
