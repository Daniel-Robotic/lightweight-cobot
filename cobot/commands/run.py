from __future__ import annotations

import argparse
import os
import signal
import shutil
import subprocess
from pathlib import Path
from typing import Callable, List, Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, LogScreen, PickScreen, RunScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent
_CONFIG_PATH = _PROJECT_DIR / "cobot-setting.yaml"
_INSTALL_DIR = _PROJECT_DIR / "install"
_JAZZY_DIR = Path("/opt/ros/jazzy")
_CONFIG_IN_CONTAINER = "/ros2_ws/cobot-setting.yaml"

_CONTAINER_CONTROLLER = "lwc-controller"
_CONTAINER_WEBOTS = "lwc-webots"

# Candidates checked in order; for controller the webots image is a valid fallback
_CONTROLLER_IMAGES = [
    "lwc-local:ros-iiwa7-jazzy",
    "evilfisru/lwc:iiwa-jazzy",
    "evilfisru/lwc:iiwa-jazzy-dev",
    "lwc-local:ros-iiwa7-webots-jazzy",
    "evilfisru/lwc:webots-jazzy",
    "evilfisru/lwc:webots-jazzy-dev",
]
_WEBOTS_IMAGES = [
    "lwc-local:ros-iiwa7-webots-jazzy",
    "evilfisru/lwc:webots-jazzy",
    "evilfisru/lwc:webots-jazzy-dev",
]


# ---------------------------------------------------------------------------
# Small utilities
# ---------------------------------------------------------------------------

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
    return _Ask(step, question, options, default).run()


def _detect_gpu() -> str:
    if shutil.which("nvidia-smi"):
        if subprocess.run(["nvidia-smi"], capture_output=True).returncode == 0:
            return "nvidia"
    if Path("/dev/dri").exists():
        return "mesa"
    return "software"


def _docker_images() -> set:
    r = subprocess.run(
        ["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
        capture_output=True, text=True,
    )
    return set(r.stdout.strip().splitlines())


def _find_image(candidates: List[str]) -> Optional[str]:
    available = _docker_images()
    for img in candidates:
        if img in available:
            return img
    return None


# ---------------------------------------------------------------------------
# Build project (colcon build --mixin release)
# ---------------------------------------------------------------------------

def _task_build(screen: LogScreen) -> None:
    try:
        screen.write("[bold]Building project with colcon[/bold]\n")

        list_proc = subprocess.run(
            ["bash", "-c", f"source {_JAZZY_DIR}/setup.bash && colcon list"],
            capture_output=True, text=True, cwd=_PROJECT_DIR,
        )
        total = max(len([l for l in list_proc.stdout.splitlines() if l.strip()]), 1)
        screen.set_progress(0, f"0 / {total} packages done")
        built = 0

        proc = subprocess.Popen(
            ["bash", "-c",
             f"source {_JAZZY_DIR}/setup.bash && colcon build --mixin release"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, cwd=_PROJECT_DIR,
        )
        for line in proc.stdout:
            s = line.rstrip()
            if s:
                screen.write(s)
            if "Finished <<<" in line or "Failed <<<" in line:
                built += 1
                screen.set_progress(built / total * 100, f"{built} / {total} packages done")
        proc.wait()

        if proc.returncode != 0:
            screen.write("\n[red]Build failed.[/red]")
            screen.finish(False)
            return

        screen.set_progress(100, "Build complete")
        screen.write("\n[green]Build successful.[/green]")
        screen.finish(True)
    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)


class _BuildApp(App[bool]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            LogScreen("Building project before launch", _task_build, show_progress=True),
            self.exit,
        )


# ---------------------------------------------------------------------------
# Local launch task
# ---------------------------------------------------------------------------

def _task_run_local(screen: RunScreen, mode: str) -> None:
    config = str(_CONFIG_PATH)
    ros_cmd = f"ros2 launch iiwa_bringup iiwa.launch.py setting:={config}"
    if mode == "webots":
        ros_cmd += " simulate:=1"

    full_cmd = (
        f"source {_JAZZY_DIR}/setup.bash && "
        f"source {_INSTALL_DIR}/setup.bash && "
        f"{ros_cmd}"
    )

    label = "Webots simulator" if mode == "webots" else "Controller"
    screen.write(f"[bold]Launching {label} (local)[/bold]")
    screen.write(f"[dim]{ros_cmd}[/dim]\n")

    proc = subprocess.Popen(
        ["bash", "-c", full_cmd],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, cwd=_PROJECT_DIR,
        start_new_session=True,
    )
    screen.set_proc(proc)
    screen.set_kill_fn(lambda: os.killpg(os.getpgid(proc.pid), signal.SIGTERM))

    for line in proc.stdout:
        s = line.rstrip()
        if s:
            screen.write(s)

    proc.wait()
    screen.finish(stopped=screen._stopped)


# ---------------------------------------------------------------------------
# Docker launch task
# ---------------------------------------------------------------------------

def _task_run_docker(screen: RunScreen, image: str, mode: str, gpu: str) -> None:
    container = _CONTAINER_WEBOTS if mode == "webots" else _CONTAINER_CONTROLLER

    ros_cmd = f"ros2 launch iiwa_bringup iiwa.launch.py setting:={_CONFIG_IN_CONTAINER}"
    if mode == "webots":
        ros_cmd += " simulate:=1"

    # Remove stale container with the same name
    subprocess.run(["docker", "rm", "-f", container], capture_output=True)

    cmd = [
        "docker", "run", "--rm",
        "--name", container,
        "--network", "host",
        "-e", "USER=root",
    ]

    if mode == "webots":
        subprocess.run(["xhost", "+local:docker"], capture_output=True)
        cmd += [
            "-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
            "-e", "QT_X11_NO_MITSHM=1",
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
        ]
        if gpu == "nvidia":
            cmd += [
                "--gpus", "all",
                "-e", "NVIDIA_VISIBLE_DEVICES=all",
                "-e", "NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute",
            ]
        elif gpu == "mesa":
            cmd += ["--device", "/dev/dri"]
        else:
            cmd += [
                "-e", "LIBGL_ALWAYS_SOFTWARE=1",
                "-e", "GALLIUM_DRIVER=llvmpipe",
            ]

    if _CONFIG_PATH.exists():
        cmd += ["-v", f"{_CONFIG_PATH}:{_CONFIG_IN_CONTAINER}:ro"]

    cmd += [image, "bash", "-c", ros_cmd]

    _GPU_LABELS = {
        "nvidia": "NVIDIA GPU",
        "mesa":   "Intel/AMD DRI (Mesa)",
        "software": "Software rendering (llvmpipe)",
    }
    label = "Webots simulator" if mode == "webots" else "Controller"
    screen.write(f"[bold]Launching {label} in Docker[/bold]")
    screen.write(f"[dim]Image: {image}[/dim]")
    if mode == "webots":
        screen.write(f"[dim]GPU:   {_GPU_LABELS.get(gpu, gpu)}[/dim]")
    screen.write("")

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True,
    )
    screen.set_proc(proc)

    for line in proc.stdout:
        s = line.rstrip()
        if s:
            screen.write(s)

    proc.wait()
    screen.finish(stopped=screen._stopped)


# ---------------------------------------------------------------------------
# RunApp wrapper
# ---------------------------------------------------------------------------

class _RunApp(App[None]):
    CSS = SCREEN_CSS

    def __init__(self, title: str, task: Callable):
        super().__init__()
        self._title = title
        self._run_fn = task

    def on_mount(self) -> None:
        self.push_screen(RunScreen(self._title, self._run_fn), lambda _: self.exit())


# ---------------------------------------------------------------------------
# Local flow
# ---------------------------------------------------------------------------

def _local_flow(args: argparse.Namespace) -> None:
    mode_v = _ask(
        "Run local",
        "What do you want to launch?",
        ["Controller", "Webots simulator"],
        "Controller",
    )
    if mode_v is None:
        return
    mode = "webots" if mode_v == "Webots simulator" else "controller"

    # Check ROS2 Jazzy
    if not _JAZZY_DIR.is_dir():
        v = _ask(
            "ROS2 not found",
            "ROS2 Jazzy is not installed. Run local-setup now?",
            ["Yes, run local-setup", "No, cancel"],
            "Yes, run local-setup",
        )
        if v and v.startswith("Yes"):
            from cobot.commands.local_setup import run as _local_setup
            _local_setup(args)
        return

    # Check project built
    if not (_INSTALL_DIR / "setup.bash").exists():
        v = _ask(
            "Project not built",
            "The project has not been built yet. Build it now?",
            ["Yes, build now", "No, cancel"],
            "Yes, build now",
        )
        if v is None or v.startswith("No"):
            return
        ok = _BuildApp().run()
        if not ok:
            return

    label = "Webots simulator" if mode == "webots" else "Controller"
    _RunApp(f"Running {label} — local", lambda s: _task_run_local(s, mode)).run()


# ---------------------------------------------------------------------------
# Docker flow
# ---------------------------------------------------------------------------

def _docker_flow(args: argparse.Namespace) -> None:
    if not shutil.which("docker"):
        from rich.console import Console
        Console().print("[red]Error:[/red] Docker is not installed or not on PATH.")
        return

    mode_v = _ask(
        "Run in Docker",
        "What do you want to launch?",
        ["Controller", "Webots simulator"],
        "Controller",
    )
    if mode_v is None:
        return
    mode = "webots" if mode_v == "Webots simulator" else "controller"

    candidates = _WEBOTS_IMAGES if mode == "webots" else _CONTROLLER_IMAGES
    image = _find_image(candidates)

    if image is None:
        what = "Webots" if mode == "webots" else "controller or Webots"
        v = _ask(
            "No image found",
            f"No Docker image found for {what}. Run docker-setup now?",
            ["Yes, run docker-setup", "No, cancel"],
            "Yes, run docker-setup",
        )
        if v and v.startswith("Yes"):
            from cobot.commands.docker_setup import run as _docker_setup
            _docker_setup(args)
        return

    gpu = _detect_gpu() if mode == "webots" else "software"

    label = "Webots simulator" if mode == "webots" else "Controller"
    _RunApp(
        f"Running {label} — Docker",
        lambda s: _task_run_docker(s, image, mode, gpu),
    ).run()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser(
        "run",
        help="Launch the robot controller or Webots simulator",
    )
    p.add_argument(
        "mode",
        nargs="?",
        choices=["local", "docker"],
        default=None,
        help="local — native ROS2, docker — Docker container (default: ask)",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    mode = getattr(args, "mode", None)

    if mode == "local":
        _local_flow(args)
    elif mode == "docker":
        _docker_flow(args)
    else:
        v = _ask(
            "Run",
            "How do you want to run the project?",
            ["Local (native ROS2)", "Docker"],
            "Local (native ROS2)",
        )
        if v is None:
            return
        if v.startswith("Local"):
            _local_flow(args)
        else:
            _docker_flow(args)
