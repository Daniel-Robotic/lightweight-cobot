from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path
from typing import Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, LogScreen, PickScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent


# Stop and remove all Docker containers whose name contains "lwc".
# Останавливаем и удаляем все Docker-контейнеры, чьё имя содержит "lwc".
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


# Remove all Docker images whose repository or tag contains "lwc".
# Удаляем все Docker-образы, репозиторий или тег которых содержит "lwc".
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


# Remove the Docker volume that stores the Webots asset cache.
# Удаляем Docker volume с кэшем ассетов Webots.
def _remove_webots_volume(write) -> None:
    result = subprocess.run(
        ["docker", "volume", "inspect", "lwc-webots-cache"],
        capture_output=True,
    )
    if result.returncode != 0:
        write("[dim]Webots cache volume not found, skipping.[/dim]")
        return
    subprocess.run(["docker", "volume", "rm", "lwc-webots-cache"], capture_output=True)
    write("[green][ok][/green] Removed Docker volume: lwc-webots-cache")


# Remove ROS2 Jazzy packages via apt and clean up the source line from shell configs.
# Uses the official removal commands to also unregister the ROS2 apt repository.
# Удаляем пакеты ROS2 Jazzy через apt и очищаем строку source из конфигов оболочки.
# Используем официальные команды удаления, которые также снимают регистрацию apt-репозитория ROS2.
def _remove_ros2(write) -> None:
    write("[cyan][*][/cyan] Removing ROS2 Jazzy packages...")
    if not Path("/opt/ros/jazzy").exists():
        write("[dim]ROS2 Jazzy not found, skipping.[/dim]")
    else:
        # Remove all ros-jazzy-* packages matched by the apt regex pattern ~n<name>.
        # Удаляем все пакеты ros-jazzy-* по regex-паттерну apt ~n<имя>.
        subprocess.run(
            ["sudo", "apt", "remove", "-y", "~nros-jazzy-*"],
            capture_output=True,
        )
        subprocess.run(["sudo", "apt", "autoremove", "-y"], capture_output=True)
        write("[green][ok][/green] ROS2 Jazzy packages removed")

        # Remove the ROS2 apt source package that added the repository.
        # Удаляем пакет apt-источника ROS2, который добавил репозиторий.
        subprocess.run(
            ["sudo", "apt", "remove", "-y", "ros2-apt-source"],
            capture_output=True,
        )
        subprocess.run(["sudo", "apt", "update", "-qq"], capture_output=True)
        subprocess.run(["sudo", "apt", "autoremove", "-y"], capture_output=True)
        write("[green][ok][/green] ROS2 apt repository removed")

    # Clean up the source line that local-setup added to the shell config.
    # Очищаем строку source, добавленную local-setup в конфиг оболочки.
    source_line = "source /opt/ros/jazzy/setup.bash"
    for rc_name in [".bashrc", ".zshrc"]:
        rc = Path.home() / rc_name
        if not rc.exists():
            continue
        content = rc.read_text()
        if source_line not in content:
            continue
        # Remove the whole block that was added by local-setup, not just the single line.
        # Удаляем весь блок добавленный local-setup, а не только одну строку.
        new_content = content.replace(f"\n# ROS2 Jazzy\n{source_line}\n", "\n")
        new_content = new_content.replace(source_line, "")
        rc.write_text(new_content)
        write(f"[green][ok][/green] Cleaned up ~/{rc_name}")


# Remove Webots from the system via apt.
# Удаляем Webots из системы через apt.
def _remove_webots(write) -> None:
    write("[cyan][*][/cyan] Removing Webots...")
    if not shutil.which("webots"):
        write("[dim]Webots not found, skipping.[/dim]")
        return
    subprocess.run(["sudo", "apt", "remove", "-y", "webots"], capture_output=True)
    subprocess.run(["sudo", "apt", "autoremove", "-y"], capture_output=True)
    write("[green][ok][/green] Webots removed")


# Uninstall the cobot CLI from the uv tool store.
# Удаляем cobot CLI из хранилища инструментов uv.
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


# Delete the entire project directory from disk.
# Удаляем всю директорию проекта с диска.
def _remove_project_dir(write) -> None:
    write(f"[cyan][*][/cyan] Removing project directory...")
    try:
        shutil.rmtree(_PROJECT_DIR)
        write(f"[green][ok][/green] Removed {_PROJECT_DIR}")
    except Exception as exc:
        write(f"[red]Failed:[/red] {exc}")
        raise


# Run all deletion steps in order.
# Progress ranges are split evenly across the active steps so the bar always reaches 100%.
# Выполняем все шаги удаления по порядку.
# Диапазоны прогресса делятся равномерно между активными шагами, чтобы бар всегда доходил до 100%.
def _task_delete(screen: LogScreen, remove_ros: bool, remove_webots: bool) -> None:
    try:
        screen.set_progress(0, "Stopping containers...")
        _stop_docker_containers(screen.write)
        _remove_webots_volume(screen.write)

        screen.set_progress(20, "Removing Docker images...")
        _remove_docker_images(screen.write)

        pct = 40
        if remove_ros:
            screen.set_progress(pct, "Removing ROS2 Jazzy...")
            _remove_ros2(screen.write)
            pct = 65

        if remove_webots:
            screen.set_progress(pct, "Removing Webots...")
            _remove_webots(screen.write)
            pct = 75

        screen.set_progress(pct, "Uninstalling cobot CLI...")
        _uninstall_cobot(screen.write)

        screen.set_progress(88, "Removing project directory...")
        _remove_project_dir(screen.write)

        if not screen.is_stopped():
            screen.set_progress(100, "Done")
            screen.write("\n[green]Project fully removed.[/green]")
            screen.finish(True)

    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


# Multi-step confirmation wizard before anything is deleted.
# Shows extra questions only when the relevant software is actually installed.
# Многошаговый мастер подтверждения перед удалением.
# Дополнительные вопросы показываются только если соответствующее ПО действительно установлено.
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
                "Also remove ROS2 Jazzy from the system?",
                ["No, keep ROS2", "Yes, remove ROS2 Jazzy"],
                "No, keep ROS2",
            ),
            self._on_ros_choice,
        )

    def _on_ros_choice(self, choice: Optional[str]) -> None:
        remove_ros = choice is not None and choice.startswith("Yes")
        # Only ask about Webots if it is actually installed on this machine.
        # Спрашиваем про Webots только если он действительно установлен на этой машине.
        if shutil.which("webots"):
            self.push_screen(
                PickScreen(
                    "Webots",
                    "Also remove Webots from the system?",
                    ["No, keep Webots", "Yes, remove Webots"],
                    "No, keep Webots",
                ),
                lambda c: self._on_webots_choice(c, remove_ros),
            )
        else:
            self._start_deletion(remove_ros, remove_webots=False)

    def _on_webots_choice(self, choice: Optional[str], remove_ros: bool) -> None:
        remove_webots = choice is not None and choice.startswith("Yes")
        self._start_deletion(remove_ros, remove_webots)

    def _start_deletion(self, remove_ros: bool, remove_webots: bool) -> None:
        self.push_screen(
            LogScreen(
                "Deleting project",
                lambda s: _task_delete(s, remove_ros, remove_webots),
                show_progress=True,
            ),
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
