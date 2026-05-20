from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path
from typing import Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, LogScreen, PickScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent


def _stop_docker_containers(write) -> None:
    write("[cyan][*][/cyan] Stopping Docker containers...")
    result = subprocess.run(
        ["docker", "ps", "-a", "--filter", "name=lwc", "--format", "{{.Names}}"],
        capture_output=True, text=True,
    )
    containers = [c for c in result.stdout.strip().splitlines() if c]
    if not containers:
        write("[dim]No project containers found.[/dim]")
        return
    for name in containers:
        subprocess.run(["docker", "rm", "-f", name], capture_output=True)
        write(f"[green][ok][/green] Removed container: {name}")


def _remove_docker_images(write) -> None:
    write("[cyan][*][/cyan] Removing Docker images...")
    result = subprocess.run(
        ["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
        capture_output=True, text=True,
    )
    project_images = [
        img for img in result.stdout.strip().splitlines()
        if "lwc" in img.lower()
    ]
    if not project_images:
        write("[dim]No project images found.[/dim]")
        return
    for img in project_images:
        subprocess.run(["docker", "rmi", "-f", img], capture_output=True)
        write(f"[green][ok][/green] Removed image: {img}")


def _remove_ros2(write) -> None:
    write("[cyan][*][/cyan] Removing ROS2 Jazzy...")
    if Path("/opt/ros/jazzy").exists():
        subprocess.run(["sudo", "rm", "-rf", "/opt/ros/jazzy"])
        write("[green][ok][/green] Removed /opt/ros/jazzy")
    else:
        write("[dim]ROS2 Jazzy not found, skipping.[/dim]")

    source_line = "source /opt/ros/jazzy/setup.bash"
    for rc_name in [".bashrc", ".zshrc"]:
        rc = Path.home() / rc_name
        if not rc.exists():
            continue
        content = rc.read_text()
        if source_line not in content:
            continue
        new_content = content.replace(f"\n# ROS2 Jazzy\n{source_line}\n", "\n")
        new_content = new_content.replace(source_line, "")
        rc.write_text(new_content)
        write(f"[green][ok][/green] Cleaned up ~/{rc_name}")


def _uninstall_cobot(write) -> None:
    write("[cyan][*][/cyan] Uninstalling cobot CLI...")
    result = subprocess.run(
        ["uv", "tool", "uninstall", "lightweight-cobot"],
        capture_output=True, text=True,
    )
    if result.returncode == 0:
        write("[green][ok][/green] cobot uninstalled")
    else:
        write(f"[yellow]Warning:[/yellow] {result.stderr.strip() or 'could not uninstall cobot'}")


def _remove_project_dir(write) -> None:
    write(f"[cyan][*][/cyan] Removing project directory...")
    try:
        shutil.rmtree(_PROJECT_DIR)
        write(f"[green][ok][/green] Removed {_PROJECT_DIR}")
    except Exception as exc:
        write(f"[red]Failed:[/red] {exc}")
        raise



def _task_delete(screen: LogScreen, remove_ros: bool) -> None:
    try:
        _stop_docker_containers(screen.write)
        _remove_docker_images(screen.write)

        if remove_ros:
            _remove_ros2(screen.write)

        _uninstall_cobot(screen.write)
        _remove_project_dir(screen.write)

        screen.write("\n[green]Project fully removed.[/green]")
        screen.finish(True)

    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)



class _DeleteApp(App[None]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(
                "Confirm deletion",
                "This will permanently delete the project, Docker images and containers. Are you sure?",
                ["No, cancel", "Yes, delete everything"],
                "No, cancel",
            ),
            self._on_confirm,
        )

    def _on_confirm(self, choice: Optional[str]) -> None:
        if choice is None or choice.startswith("No"):
            self.exit()
            return
        self.push_screen(
            PickScreen(
                "ROS2 Jazzy",
                "Also remove ROS2 Jazzy (/opt/ros/jazzy)?",
                ["No, keep ROS2", "Yes, remove ROS2 Jazzy"],
                "No, keep ROS2",
            ),
            self._on_ros_choice,
        )

    def _on_ros_choice(self, choice: Optional[str]) -> None:
        remove_ros = choice is not None and choice.startswith("Yes")
        self.push_screen(
            LogScreen("Deleting project", lambda s: _task_delete(s, remove_ros)),
            lambda _: self.exit(),
        )


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser(
        "delete",
        help="Remove the project, Docker images, containers, and optionally ROS2",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    _DeleteApp().run()
