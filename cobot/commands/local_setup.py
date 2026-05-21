from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
import threading
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


def _run_quiet(cmd: List[str], write: Write | None = None, env: dict | None = None, cwd=None) -> None:
    result = subprocess.run(
        cmd, capture_output=True, text=True,
        env=env or os.environ, cwd=cwd,
    )
    if result.returncode != 0:
        if write:
            for line in (result.stdout + result.stderr).splitlines():
                if line.strip():
                    write(line)
        raise RuntimeError(f"Command failed: {cmd[0]}")


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
        raise RuntimeError(f"Command failed: {cmd[0]}")


def _run_apt_with_progress(
    cmd: List[str],
    write: Write,
    on_progress: Callable[[float], None],
    env: dict | None = None,
) -> None:
    """Run an apt command and feed real percentage from APT::Status-Fd to on_progress(0-100)."""
    r_fd, w_fd = os.pipe()
    try:
        proc = subprocess.Popen(
            cmd + [f"-o", f"APT::Status-Fd={w_fd}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env or os.environ,
            pass_fds=(w_fd,),
        )
    finally:
        os.close(w_fd)

    def _read_status() -> None:
        with os.fdopen(r_fd, "r") as f:
            for line in f:
                # Format: dlstatus:N:PCT:MSG  or  pmstatus:NAME:PCT:MSG
                parts = line.strip().split(":", 3)
                if len(parts) >= 3:
                    try:
                        on_progress(float(parts[2]))
                    except ValueError:
                        pass

    t = threading.Thread(target=_read_status, daemon=True)
    t.start()
    for line in proc.stdout:
        s = line.rstrip()
        if s:
            write(s)
    proc.wait()
    t.join()
    if proc.returncode != 0:
        raise RuntimeError(f"Command failed: {cmd[0]}")


# ---------------------------------------------------------------------------
# Installation steps
# ---------------------------------------------------------------------------

def _setup_locale(write: Write) -> None:
    write("[cyan][*][/cyan] Checking locale...")
    if "UTF-8" in subprocess.run(["locale"], capture_output=True, text=True).stdout:
        write("[green][ok][/green] UTF-8 locale active")
        return
    write("[cyan][*][/cyan] Configuring UTF-8 locale...")
    _run_quiet(["sudo", "apt-get", "update", "-qq"], write)
    _run_quiet(["sudo", "apt-get", "install", "-y", "--no-install-recommends", "locales"], write, _APT_ENV)
    _run_quiet(["sudo", "locale-gen", "en_US.UTF-8"], write)
    _run_quiet(["sudo", "update-locale", "LC_ALL=en_US.UTF-8", "LANG=en_US.UTF-8"], write)
    write("[green][ok][/green] Locale configured")


def _add_ros2_repo(write: Write, on_progress: Optional[Callable[[float], None]] = None) -> None:
    def _prog(p: float) -> None:
        if on_progress:
            on_progress(p)

    write("[cyan][*][/cyan] Adding ROS2 apt repository...")

    # Best-effort update before installing prereqs
    subprocess.run(["sudo", "apt-get", "update", "-qq"], capture_output=True)
    _prog(15)

    _run_quiet(
        ["sudo", "apt-get", "install", "-y", "--no-install-recommends",
         "software-properties-common", "curl", "gnupg"],
        write, _APT_ENV,
    )
    _prog(35)
    _run_quiet(["sudo", "add-apt-repository", "-y", "universe"], write)
    _prog(45)

    write("[cyan][*][/cyan] Downloading ROS2 signing key...")
    with tempfile.NamedTemporaryFile(delete=False, suffix=".key") as tmp:
        tmp_path = tmp.name
    try:
        urllib.request.urlretrieve(_ROS_KEY_URL, tmp_path)
        _prog(60)
        _run_quiet(["sudo", "gpg", "--dearmor", "--yes", "-o", str(_ROS_KEYRING), tmp_path])
    finally:
        os.unlink(tmp_path)
    write("[green][ok][/green] Signing key installed")
    _prog(65)

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
    _prog(70)

    write("[cyan][*][/cyan] Updating apt cache...")
    _run_apt_with_progress(
        ["sudo", "apt-get", "update", "-q"],
        write,
        lambda p: _prog(70 + p * 0.30),
        _APT_ENV,
    )
    write("[green][ok][/green] ROS2 repository ready")
    _prog(100)


def _install_ros2_jazzy(write: Write, on_progress: Optional[Callable[[float], None]] = None) -> None:
    write("[cyan][*][/cyan] Installing ros-jazzy-desktop and ros-dev-tools...")
    _run_apt_with_progress(
        ["sudo", "apt-get", "install", "-y", "ros-jazzy-desktop", "ros-dev-tools"],
        write,
        on_progress or (lambda _: None),
        _APT_ENV,
    )
    write("[green][ok][/green] ROS2 Jazzy Desktop installed")


def _install_colcon(write: Write) -> None:
    if shutil.which("colcon"):
        write("[green][ok][/green] colcon already available")
        return
    write("[cyan][*][/cyan] Installing colcon...")
    _run_quiet(
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


# ---------------------------------------------------------------------------
# Background tasks (run inside LogScreen worker)
# ---------------------------------------------------------------------------

def _task_install_jazzy(screen: LogScreen) -> None:
    try:
        # Step 1 — locale  (0 → 5 %)
        screen.set_progress(0, "Setting up locale...")
        screen.write("[bold]Step 1 / 5 — Locale[/bold]")
        _setup_locale(screen.write)

        # Step 2 — ROS2 repo  (5 → 20 %)
        screen.set_progress(5, "Adding ROS2 repository...")
        screen.write("\n[bold]Step 2 / 5 — ROS2 repository[/bold]")
        _add_ros2_repo(
            screen.write,
            on_progress=lambda p: screen.set_progress(5 + p * 0.15),
        )

        # Step 3 — ROS2 Jazzy  (20 → 85 %)
        screen.set_progress(20, "Installing ROS2 Jazzy Desktop...")
        screen.write("\n[bold]Step 3 / 5 — ROS2 Jazzy Desktop[/bold]")
        _install_ros2_jazzy(
            screen.write,
            on_progress=lambda p: screen.set_progress(20 + p * 0.65),
        )

        # Step 4 — colcon  (85 → 92 %)
        screen.set_progress(85, "Installing colcon...")
        screen.write("\n[bold]Step 4 / 5 — colcon[/bold]")
        _install_colcon(screen.write)

        # Step 5 — shell rc  (92 → 100 %)
        screen.set_progress(92, "Configuring shell...")
        screen.write("\n[bold]Step 5 / 5 — Shell configuration[/bold]")
        _setup_shell_rc(screen.write)
        screen.set_progress(100, "Done")

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

        # Count packages so we can show X/total progress
        list_result = subprocess.run(
            ["colcon", "list"], capture_output=True, text=True, cwd=_PROJECT_DIR,
        )
        total = max(len([l for l in list_result.stdout.splitlines() if l.strip()]), 1)

        screen.write(f"[bold]Building {total} package(s) with colcon[/bold]\n")
        screen.set_progress(0, f"0 / {total} packages done")
        built = 0

        def _track(line: str) -> None:
            nonlocal built
            screen.write(line)
            if "Finished <<<" in line or "Failed <<<" in line:
                built += 1
                screen.set_progress(built / total * 100, f"{built} / {total} packages done")

        _run_logged(["colcon", "build", "--symlink-install"], _track, cwd=_PROJECT_DIR)

        screen.set_progress(100, "Build complete")
        screen.write("\nActivate workspace:  [bold]source install/setup.bash[/bold]")
        screen.finish(True)
    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)


# ---------------------------------------------------------------------------
# Textual apps
# ---------------------------------------------------------------------------

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
            LogScreen("Installing ROS2 Jazzy", _task_install_jazzy, show_progress=True),
            lambda _: self.exit(),
        )


class _BuildApp(App[None]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            LogScreen("Building project", _task_build, show_progress=True),
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


# ---------------------------------------------------------------------------
# CLI registration
# ---------------------------------------------------------------------------

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
