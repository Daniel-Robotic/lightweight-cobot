from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
import urllib.request
from pathlib import Path
from typing import Callable, List, Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, LogScreen, PickScreen
from cobot.commands.docker_setup import run as _docker_setup

_PROJECT_DIR = Path(__file__).parent.parent.parent

_ROS_KEYRING = Path("/usr/share/keyrings/ros-archive-keyring.gpg")
_ROS_SOURCES = Path("/etc/apt/sources.list.d/ros2.list")
_ROS_KEY_URL = "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
_APT_ENV = {**os.environ, "DEBIAN_FRONTEND": "noninteractive"}


def _detect_ubuntu_2404() -> bool:
    path = Path("/etc/os-release")
    if not path.exists():
        return False
    info: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            info[k.strip()] = v.strip().strip('"')
    return info.get("ID") == "ubuntu" and info.get("VERSION_ID") == "24.04"


def _detect_ros2_jazzy() -> bool:
    return Path("/opt/ros/jazzy").is_dir()


Write = Callable[[str], None]


def _run_quiet(cmd: List[str]) -> None:
    result = subprocess.run(cmd, capture_output=True)
    if result.returncode != 0:
        raise RuntimeError(f"Exit {result.returncode}: {cmd[0]}")


def _run_logged(cmd: List[str], write: Write, env: dict | None = None, cwd=None) -> None:
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env or os.environ,
        cwd=cwd,
    )
    for line in proc.stdout:
        s = line.rstrip()
        if s:
            write(s)
    proc.wait()
    if proc.returncode != 0:
        raise RuntimeError(f"Exit {proc.returncode}: {cmd[0]}")


def _setup_locale(write: Write) -> None:
    write("[cyan][*][/cyan] Checking locale...")
    if "UTF-8" in subprocess.run(["locale"], capture_output=True, text=True).stdout:
        write("[green][ok][/green] UTF-8 locale active")
        return
    write("[cyan][*][/cyan] Configuring UTF-8 locale...")
    _run_quiet(["sudo", "apt-get", "update", "-qq"])
    _run_logged(["sudo", "apt-get", "install", "-y", "--no-install-recommends", "locales"], write, _APT_ENV)
    _run_logged(["sudo", "locale-gen", "en_US.UTF-8"], write)
    _run_quiet(["sudo", "update-locale", "LC_ALL=en_US.UTF-8", "LANG=en_US.UTF-8"])
    write("[green][ok][/green] Locale configured")


def _add_ros2_repo(write: Write) -> None:
    write("[cyan][*][/cyan] Adding ROS2 apt repository...")
    _run_quiet(["sudo", "apt-get", "update", "-qq"])
    _run_logged(
        ["sudo", "apt-get", "install", "-y", "--no-install-recommends",
         "software-properties-common", "curl"],
        write, _APT_ENV,
    )
    _run_quiet(["sudo", "add-apt-repository", "-y", "universe"])

    if not _ROS_KEYRING.exists():
        write("[cyan][*][/cyan] Downloading ROS2 signing key...")
        with tempfile.NamedTemporaryFile(delete=False, suffix=".gpg") as tmp:
            tmp_path = tmp.name
        try:
            urllib.request.urlretrieve(_ROS_KEY_URL, tmp_path)
            _run_quiet(["sudo", "cp", tmp_path, str(_ROS_KEYRING)])
        finally:
            os.unlink(tmp_path)

    if not _ROS_SOURCES.exists():
        arch = subprocess.check_output(["dpkg", "--print-architecture"], text=True).strip()
        codename = subprocess.check_output(
            ["bash", "-c", ". /etc/os-release && echo $UBUNTU_CODENAME"], text=True
        ).strip()
        sources_line = (
            f"deb [arch={arch} signed-by={_ROS_KEYRING}] "
            f"http://packages.ros.org/ros2/ubuntu {codename} main\n"
        )
        proc = subprocess.run(
            ["sudo", "tee", str(_ROS_SOURCES)],
            input=sources_line, capture_output=True, text=True,
        )
        if proc.returncode != 0:
            raise RuntimeError(f"Failed to write {_ROS_SOURCES}")

    _run_quiet(["sudo", "apt-get", "update", "-qq"])
    write("[green][ok][/green] ROS2 repository ready")


def _install_ros2_jazzy(write: Write) -> None:
    write("[cyan][*][/cyan] Installing ros-jazzy-ros-base and ros-dev-tools...")
    _run_logged(
        ["sudo", "apt-get", "install", "-y", "--no-install-recommends",
         "ros-jazzy-ros-base", "ros-dev-tools"],
        write, _APT_ENV,
    )
    write("[green][ok][/green] ROS2 Jazzy installed")


def _install_colcon(write: Write) -> None:
    if shutil.which("colcon"):
        write("[green][ok][/green] colcon already available")
        return
    write("[cyan][*][/cyan] Installing colcon...")
    _run_logged(
        ["sudo", "apt-get", "install", "-y", "--no-install-recommends",
         "python3-colcon-common-extensions"],
        write, _APT_ENV,
    )
    write("[green][ok][/green] colcon installed")


def _setup_shell_rc(write: Write) -> None:
    shell_name = Path(os.environ.get("SHELL", "/bin/bash")).name
    rc = Path.home() / (".zshrc" if shell_name == "zsh" else ".bashrc")
    source_line = "source /opt/ros/jazzy/setup.bash"
    if rc.exists() and source_line in rc.read_text():
        write(f"[green][ok][/green] ROS2 setup already in {rc.name}")
        return
    with rc.open("a") as f:
        f.write(f"\n# ROS2 Jazzy\n{source_line}\n")
    write(f"[green][ok][/green] Added ROS2 setup to ~/{rc.name}")


def _task_install_jazzy(screen: LogScreen) -> None:
    try:
        screen.write("[bold]Installing ROS2 Jazzy[/bold]\n")
        _setup_locale(screen.write)
        _add_ros2_repo(screen.write)
        _install_ros2_jazzy(screen.write)
        _install_colcon(screen.write)
        _setup_shell_rc(screen.write)
        screen.write(
            "\nRestart the terminal, then run [bold]cobot local-setup[/bold] again to build."
        )
        screen.finish(True)
    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)


def _task_build(screen: LogScreen) -> None:
    try:
        if not shutil.which("colcon"):
            screen.write("[red]colcon not found.[/red]")
            screen.write("Source ROS2 first:  [bold]source /opt/ros/jazzy/setup.bash[/bold]")
            screen.finish(False)
            return
        screen.write("[bold]Building project with colcon[/bold]\n")
        _run_logged(
            ["colcon", "build", "--symlink-install"],
            screen.write,
            cwd=_PROJECT_DIR,
        )
        screen.write("\nActivate workspace:  [bold]source install/setup.bash[/bold]")
        screen.finish(True)
    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)


class _InstallJazzyApp(App[None]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(
                "ROS2 not found",
                "ROS2 Jazzy is not installed. Install it now?",
                ["Yes, install ROS2 Jazzy", "No, skip"],
                "Yes, install ROS2 Jazzy",
            ),
            self._on_choice,
        )

    def _on_choice(self, choice: Optional[str]) -> None:
        if choice is None or choice.startswith("No"):
            self.exit()
            return
        self.push_screen(
            LogScreen("Installing ROS2 Jazzy", _task_install_jazzy),
            lambda _: self.exit(),
        )


class _BuildApp(App[None]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            LogScreen("Building project", _task_build),
            lambda _: self.exit(),
        )


class _DockerPromptApp(App[bool]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(
                "Unsupported OS",
                "Ubuntu 24.04 not detected. Build a Docker image for development?",
                ["Yes, run docker-setup", "No, exit"],
                "Yes, run docker-setup",
            ),
            lambda v: self.exit(v is not None and v.startswith("Yes")),
        )


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser(
        "local-setup",
        help="Install ROS2 Jazzy natively and build the project with colcon",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    if not _detect_ubuntu_2404():
        if _DockerPromptApp().run():
            _docker_setup(args)
        return

    if not _detect_ros2_jazzy():
        _InstallJazzyApp().run()
        return

    _BuildApp().run()
