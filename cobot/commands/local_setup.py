from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path
from typing import List, Optional

from rich.console import Console
from textual.app import App

from cobot.tui import SCREEN_CSS, PickScreen
from cobot.commands.docker_setup import run as _docker_setup

_console = Console()
_PROJECT_DIR = Path(__file__).parent.parent.parent

_ROS_KEYRING = Path("/usr/share/keyrings/ros-archive-keyring.gpg")
_ROS_SOURCES = Path("/etc/apt/sources.list.d/ros2.list")
_ROS_KEY_URL = "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"


def _detect_ubuntu_2404() -> bool:
    os_release = Path("/etc/os-release")
    if not os_release.exists():
        return False
    info: dict[str, str] = {}
    for line in os_release.read_text().splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            info[k.strip()] = v.strip().strip('"')
    return info.get("ID") == "ubuntu" and info.get("VERSION_ID") == "24.04"


def _detect_ros2_jazzy() -> bool:
    return Path("/opt/ros/jazzy").is_dir()


def _run(cmd: List[str]) -> None:
    result = subprocess.run(cmd)
    if result.returncode != 0:
        raise RuntimeError(f"Command failed: {' '.join(cmd)}")


def _check_output(cmd: List[str]) -> str:
    return subprocess.check_output(cmd, text=True).strip()


def _step(msg: str) -> None:
    _console.print(f"[cyan][*][/cyan] {msg}")


def _ok(msg: str) -> None:
    _console.print(f"[green][ok][/green] {msg}")


def _apt_install(*packages: str) -> None:
    _run(["sudo", "apt-get", "install", "-y", "--no-install-recommends", *packages])


def _setup_locale() -> None:
    _step("Checking locale...")
    result = subprocess.run(["locale"], capture_output=True, text=True)
    if "UTF-8" in result.stdout:
        _ok("UTF-8 locale active")
        return
    _run(["sudo", "apt-get", "update", "-qq"])
    _apt_install("locales")
    _run(["sudo", "locale-gen", "en_US.UTF-8"])
    _run(["sudo", "update-locale", "LC_ALL=en_US.UTF-8", "LANG=en_US.UTF-8"])
    _ok("Locale configured")


def _add_ros2_repo() -> None:
    _step("Adding ROS2 apt repository...")
    _run(["sudo", "apt-get", "update", "-qq"])
    _apt_install("software-properties-common", "curl")
    _run(["sudo", "add-apt-repository", "-y", "universe"])

    if not _ROS_KEYRING.exists():
        _step("Downloading ROS2 signing key...")
        with tempfile.NamedTemporaryFile(delete=False, suffix=".gpg") as tmp:
            tmp_path = tmp.name
        try:
            urllib.request.urlretrieve(_ROS_KEY_URL, tmp_path)
            _run(["sudo", "cp", tmp_path, str(_ROS_KEYRING)])
        finally:
            os.unlink(tmp_path)

    if not _ROS_SOURCES.exists():
        arch = _check_output(["dpkg", "--print-architecture"])
        codename = _check_output(
            ["bash", "-c", ". /etc/os-release && echo $UBUNTU_CODENAME"]
        )
        sources_line = (
            f"deb [arch={arch} signed-by={_ROS_KEYRING}] "
            f"http://packages.ros.org/ros2/ubuntu {codename} main\n"
        )
        proc = subprocess.run(
            ["sudo", "tee", str(_ROS_SOURCES)],
            input=sources_line,
            capture_output=True,
            text=True,
        )
        if proc.returncode != 0:
            raise RuntimeError(f"Failed to write {_ROS_SOURCES}")

    _run(["sudo", "apt-get", "update", "-qq"])
    _ok("ROS2 repository ready")


def _install_ros2_jazzy() -> None:
    _step("Installing ros-jazzy-ros-base + ros-dev-tools...")
    _apt_install("ros-jazzy-ros-base", "ros-dev-tools")
    _ok("ROS2 Jazzy installed")


def _install_colcon() -> None:
    if shutil.which("colcon"):
        _ok("colcon already available")
        return
    _step("Installing colcon...")
    _apt_install("python3-colcon-common-extensions")
    _ok("colcon installed")


def _setup_shell_rc() -> None:
    shell_name = Path(os.environ.get("SHELL", "/bin/bash")).name
    rc = Path.home() / (".zshrc" if shell_name == "zsh" else ".bashrc")
    source_line = "source /opt/ros/jazzy/setup.bash"
    if rc.exists() and source_line in rc.read_text():
        _ok(f"ROS2 setup already in {rc.name}")
        return
    with rc.open("a") as f:
        f.write(f"\n# ROS2 Jazzy\n{source_line}\n")
    _ok(f"Added ROS2 setup to ~/{rc.name}")


def _install_jazzy() -> None:
    _console.print("\n[bold]Installing ROS2 Jazzy...[/bold]\n")
    try:
        _setup_locale()
        _add_ros2_repo()
        _install_ros2_jazzy()
        _install_colcon()
        _setup_shell_rc()
    except (subprocess.CalledProcessError, RuntimeError) as exc:
        _console.print(f"\n[red]Installation failed:[/red] {exc}")
        sys.exit(1)
    _console.print(
        "\n[green]ROS2 Jazzy installed.[/green] "
        "Restart the terminal, then run [bold]cobot local-setup[/bold] again to build."
    )


def _build_project() -> None:
    if not shutil.which("colcon"):
        _console.print("[red]Error:[/red] colcon not found. Source ROS2 first:")
        _console.print("  [bold]source /opt/ros/jazzy/setup.bash[/bold]")
        sys.exit(1)

    _console.print("\n[bold]Building project with colcon...[/bold]\n")
    result = subprocess.run(
        ["colcon", "build", "--symlink-install"],
        cwd=_PROJECT_DIR,
    )
    if result.returncode != 0:
        _console.print("\n[red]Build failed.[/red]")
        sys.exit(result.returncode)
    _console.print("\n[green]Build complete.[/green]")
    _console.print("  Activate workspace:  [bold]source install/setup.bash[/bold]")


class _AskInstallJazzy(App[Optional[str]]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(
                "ROS2 not found",
                "ROS2 Jazzy is not installed. Install it now?",
                ["Yes, install ROS2 Jazzy", "No, skip"],
                "Yes, install ROS2 Jazzy",
            ),
            self.exit,
        )


class _AskDockerSetup(App[Optional[str]]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(
                "Unsupported OS",
                "Ubuntu 24.04 not detected. Build a Docker image for development?",
                ["Yes, run docker-setup", "No, exit"],
                "Yes, run docker-setup",
            ),
            self.exit,
        )


def _flow_ubuntu_without_ros() -> None:
    _console.print(
        "\n[yellow]Warning:[/yellow] ROS2 Jazzy is not installed "
        "(/opt/ros/jazzy not found).\n"
    )
    answer = _AskInstallJazzy().run()
    if answer is None or answer.startswith("No"):
        _console.print("[yellow]Skipped ROS2 installation.[/yellow]")
        return
    _install_jazzy()


def _flow_not_ubuntu(args: argparse.Namespace) -> None:
    _console.print(
        "\n[yellow]Warning:[/yellow] Ubuntu 24.04 not detected. "
        "Native build is not supported on this OS.\n"
    )
    answer = _AskDockerSetup().run()
    if answer is None or answer.startswith("No"):
        _console.print("[yellow]Exiting without changes.[/yellow]")
        return

    _docker_setup(args)


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser(
        "local-setup",
        help="Build the project locally (requires Ubuntu 24.04 and ROS2 Jazzy)",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    _console.print("[bold]Checking environment...[/bold]")

    if not _detect_ubuntu_2404():
        _flow_not_ubuntu(args)
        return

    _console.print("[green]  Ubuntu 24.04[/green]  ✓")

    if not _detect_ros2_jazzy():
        _flow_ubuntu_without_ros()
        return

    _console.print("[green]  ROS2 Jazzy[/green]    ✓\n")
    _build_project()
