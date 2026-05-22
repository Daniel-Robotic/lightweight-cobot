from __future__ import annotations

import argparse
import os
import signal
import shutil
import socket
import subprocess
from pathlib import Path
from typing import Callable, List, Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, LogScreen, PickScreen, RunScreen
from cobot.commands.local_setup import webots_installed, WebotsInstallApp, _WEBOTS_VERSION

_PROJECT_DIR = Path(__file__).parent.parent.parent
_CONFIG_PATH = _PROJECT_DIR / "cobot-setting.yaml"
_INSTALL_DIR = _PROJECT_DIR / "install"
_JAZZY_DIR = Path("/opt/ros/jazzy")

# Path where the config file is mounted inside the Docker container.
# Путь по которому конфиг-файл монтируется внутри Docker-контейнера.
_CONFIG_IN_CONTAINER = "/ros2_ws/cobot-setting.yaml"

# Container names used for docker run and docker kill.
# Имена контейнеров, используемые для docker run и docker kill.
_CONTAINER_CONTROLLER = "lwc-controller"
_CONTAINER_WEBOTS = "lwc-webots"

# Named Docker volume that stores the Webots asset cache between container runs.
# Without it Webots re-downloads all 3D assets from the internet on every launch.
# Именованный Docker volume для хранения кэша ассетов Webots между запусками контейнера.
# Без него Webots заново скачивает все 3D-ассеты из интернета при каждом запуске.
_WEBOTS_CACHE_VOLUME = "lwc-webots-cache"

# Candidates checked in order - for the controller the webots image is a valid fallback
# because it already contains all controller packages too.
# Кандидаты проверяются по порядку - для контроллера образ webots является допустимым запасным,
# так как он уже содержит все пакеты контроллера.
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


# A minimal app that asks one question and exits immediately with the chosen value.
# We need a full App because Textual screens cannot run outside one.
# Минимальное приложение, которое задаёт один вопрос и сразу выходит с выбранным значением.
# Нам нужен полноценный App, потому что экраны Textual не могут работать вне него.
class _Ask(App[Optional[str]]):
    """Minimal one-question Textual app. Pushes a PickScreen and exits with the chosen value.
    Минимальное однвопросное Textual-приложение. Открывает PickScreen и завершается с выбранным значением.
    """
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
    """Show a single-choice PickScreen and return the selected value, or None on Escape.
    Показывает PickScreen с одним выбором и возвращает выбранное значение или None при Escape.
    """
    # Returns None when the user pressed Escape to cancel.
    # Возвращает None когда пользователь нажал Escape для отмены.
    return _Ask(step, question, options, default).run()


# Detect the GPU type so we can pass the right flags to docker run for Webots rendering.
# Определяем тип GPU, чтобы передать нужные флаги в docker run для рендеринга Webots.
def _detect_gpu() -> str:
    """Return "nvidia", "mesa", or "software" based on what GPU drivers are available.
    Возвращает "nvidia", "mesa" или "software" в зависимости от доступных драйверов GPU.
    """
    if shutil.which("nvidia-smi"):
        if subprocess.run(["nvidia-smi"], capture_output=True).returncode == 0:
            return "nvidia"
    if Path("/dev/dri").exists():
        return "mesa"
    return "software"


# List all Docker images currently available on this machine.
# Получаем список всех Docker-образов доступных на этой машине.
def _docker_images() -> set:
    """Return the set of "repository:tag" strings for all locally available Docker images.
    Возвращает множество строк "репозиторий:тег" для всех локально доступных Docker-образов.
    """
    r = subprocess.run(
        ["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
        capture_output=True, text=True,
    )
    return set(r.stdout.strip().splitlines())


# Return the first image from the candidates list that is already present locally.
# Возвращаем первый образ из списка кандидатов, который уже присутствует локально.
def _find_image(candidates: List[str]) -> Optional[str]:
    """Return the first candidate image that exists locally, or None if none are available.
    Возвращает первый образ-кандидат, присутствующий локально, или None если ни один не найден.
    """
    available = _docker_images()
    for img in candidates:
        if img in available:
            return img
    return None


# Build the ROS2 project locally with colcon. Used when launching in local mode
# and the install/ directory does not exist yet.
# Собираем ROS2-проект локально с помощью colcon. Используется при запуске в локальном режиме,
# если директория install/ ещё не существует.
def _task_build(screen: LogScreen) -> None:
    """Worker function that runs inside LogScreen. Counts packages, then runs colcon build
    with release mixin and updates progress as each package finishes.
    Рабочая функция внутри LogScreen. Подсчитывает пакеты, запускает colcon build с mixin release
    и обновляет прогресс по мере завершения каждого пакета.
    """
    try:
        screen.write("[bold]Building project with colcon[/bold]\n")

        # Count packages first so we can show X/total progress.
        # Сначала считаем пакеты, чтобы показывать X/всего в прогрессе.
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
            # colcon prints "Finished <<<" or "Failed <<<" when each package is done.
            # colcon печатает "Finished <<<" или "Failed <<<" когда каждый пакет готов.
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
    """Minimal app that opens a LogScreen running _task_build and exits with the build result.
    Минимальное приложение, открывающее LogScreen с _task_build и завершающееся с результатом сборки.
    """

    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            LogScreen("Building project before launch", _task_build, show_progress=True),
            self.exit,
        )


# Start the ROS2 launch file directly on this machine without Docker.
# Uses start_new_session so we can kill the whole process group with one signal.
# Запускаем launch-файл ROS2 напрямую на этой машине без Docker.
# Используем start_new_session, чтобы можно было убить всю группу процессов одним сигналом.
def _task_run_local(screen: RunScreen, mode: str) -> None:
    """Worker function that runs inside RunScreen. Launches iiwa.launch.py locally by sourcing
    ROS2 and install/setup.bash, then streams output until the process exits or is stopped.
    Рабочая функция внутри RunScreen. Запускает iiwa.launch.py локально через source ROS2 и
    install/setup.bash, затем транслирует вывод до завершения процесса или его остановки.
    """
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
    # Kill the entire session so all ROS2 nodes are terminated together.
    # ros2 launch puts each node in its own process group (setpgrp), so killpg on the
    # bash pgid only reaches bash/launch itself. All nodes share the session started
    # with start_new_session=True, so pkill -s reaches every one of them.
    # Убиваем всю сессию, чтобы все ROS2-узлы завершились вместе.
    # ros2 launch помещает каждый узел в отдельную группу процессов (setpgrp), поэтому
    # killpg по pgid bash достигает только bash/launch. Все узлы разделяют сессию,
    # созданную через start_new_session=True, поэтому pkill -s достигает каждого из них.
    sid = os.getsid(proc.pid)
    screen.set_kill_fn(lambda: subprocess.run(
        ["pkill", "-TERM", "-s", str(sid)], capture_output=True
    ))

    for line in proc.stdout:
        s = line.rstrip()
        if s:
            screen.write(s)

    proc.wait()
    screen.finish(stopped=screen._stopped)


# Start the ROS2 launch file inside a Docker container.
# For Webots mode we also forward X11 and GPU access so the simulator window can appear on screen.
# Запускаем launch-файл ROS2 внутри Docker-контейнера.
# Для режима Webots также пробрасываем X11 и доступ к GPU, чтобы окно симулятора появилось на экране.
def _task_run_docker(screen: RunScreen, image: str, mode: str, gpu: str) -> None:
    """Worker function that runs inside RunScreen. Builds the docker run command with the
    appropriate GPU/X11 flags for Webots, then streams container output until stopped or exited.
    Рабочая функция внутри RunScreen. Формирует команду docker run с нужными флагами GPU/X11
    для Webots, затем транслирует вывод контейнера до остановки или завершения.
    """
    container = _CONTAINER_WEBOTS if mode == "webots" else _CONTAINER_CONTROLLER

    ros_cmd = (
        "source /ros2_ws/install/setup.bash && "
        f"ros2 launch iiwa_bringup iiwa.launch.py setting:={_CONFIG_IN_CONTAINER}"
    )
    if mode == "webots":
        ros_cmd += " simulate:=1"

    # Remove any stale container with the same name left from a previous run.
    # Удаляем устаревший контейнер с таким же именем, оставшийся от предыдущего запуска.
    subprocess.run(["docker", "rm", "-f", container], capture_output=True)

    cmd = [
        "docker", "run", "--rm",
        "--name", container,
        "--network", "host",
        "--hostname", socket.gethostname(),
        "-e", "USER=root",
    ]

    if mode == "webots":
        # Allow the container to open windows on the host display.
        # Разрешаем контейнеру открывать окна на дисплее хоста.
        subprocess.run(["xhost", "+local:docker"], capture_output=True)
        cmd += [
            "-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
            "-e", "QT_X11_NO_MITSHM=1",
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
            # Persist the Webots asset cache so it is not re-downloaded on every launch.
            # Сохраняем кэш ассетов Webots, чтобы он не скачивался заново при каждом запуске.
            "-v", f"{_WEBOTS_CACHE_VOLUME}:/root/.cache/Cyberbotics/Webots",
        ]
        if gpu == "nvidia":
            cmd += [
                "--gpus", "all",
                "-e", "NVIDIA_VISIBLE_DEVICES=all",
                "-e", "NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute",
            ]
        elif gpu == "mesa":
            # Pass through the DRI device for Intel/AMD hardware acceleration.
            # Пробрасываем DRI-устройство для аппаратного ускорения Intel/AMD.
            cmd += ["--device", "/dev/dri"]
        else:
            # No GPU found - fall back to software rendering via llvmpipe.
            # GPU не найден - используем программный рендеринг через llvmpipe.
            cmd += [
                "-e", "LIBGL_ALWAYS_SOFTWARE=1",
                "-e", "GALLIUM_DRIVER=llvmpipe",
            ]

    # Mount the config file so the container uses our local cobot-setting.yaml.
    # Монтируем конфиг-файл, чтобы контейнер использовал наш локальный cobot-setting.yaml.
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
    # Use docker kill instead of proc.terminate() so the container is stopped immediately.
    # Terminating only the docker CLI process leaves the container itself running.
    # Используем docker kill вместо proc.terminate(), чтобы контейнер остановился немедленно.
    # Завершение только процесса docker CLI оставляет сам контейнер работающим.
    screen.set_kill_fn(lambda: subprocess.run(["docker", "kill", container], capture_output=True))

    for line in proc.stdout:
        s = line.rstrip()
        if s:
            screen.write(s)

    proc.wait()
    screen.finish(stopped=screen._stopped)


# Wraps a RunScreen in an App so it can be launched with .run().
# Оборачивает RunScreen в App, чтобы его можно было запустить через .run().
class _RunApp(App[None]):
    """Minimal app that wraps a RunScreen so it can be started with .run().
    Минимальное приложение, оборачивающее RunScreen чтобы его можно было запустить через .run().
    """
    CSS = SCREEN_CSS

    def __init__(self, title: str, task: Callable):
        super().__init__()
        self._title = title
        self._run_fn = task

    def on_mount(self) -> None:
        self.push_screen(RunScreen(self._title, self._run_fn), lambda _: self.exit())


# Guide the user through launching locally - asks what to run, checks prerequisites,
# installs Webots and builds the project if needed, then launches.
# Ведёт пользователя через локальный запуск - спрашивает что запустить, проверяет
# предварительные условия, устанавливает Webots и собирает проект при необходимости, затем запускает.
def _local_flow(args: argparse.Namespace) -> None:
    """Interactive flow for local (non-Docker) launch. Checks Webots, ROS2, and build state,
    offers to install/build missing pieces, then starts RunScreen.
    Интерактивный сценарий для локального (не Docker) запуска. Проверяет Webots, ROS2 и состояние
    сборки, предлагает установить/собрать недостающее, затем запускает RunScreen.
    """
    mode_v = _ask(
        "Run local",
        "What do you want to launch?",
        ["Controller", "Webots simulator"],
        "Controller",
    )
    if mode_v is None:
        return
    mode = "webots" if mode_v == "Webots simulator" else "controller"

    # Check Webots installed (local mode only)
    if mode == "webots" and not webots_installed():
        v = _ask(
            "Webots not found",
            f"Webots {_WEBOTS_VERSION} is not installed. Install it now?",
            [f"Yes, install Webots {_WEBOTS_VERSION}", "No, cancel"],
            f"Yes, install Webots {_WEBOTS_VERSION}",
        )
        if v is None or v.startswith("No"):
            return
        ok = WebotsInstallApp().run()
        if not ok:
            return

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


# Guide the user through launching in Docker - asks what to run, finds a suitable image,
# detects the GPU for Webots, and launches.
# Ведёт пользователя через запуск в Docker - спрашивает что запустить, ищет подходящий образ,
# определяет GPU для Webots и запускает.
def _docker_flow(args: argparse.Namespace) -> None:
    """Interactive flow for Docker launch. Finds the best available image, detects the GPU
    for Webots mode, then starts RunScreen with the docker run task.
    Интерактивный сценарий для запуска в Docker. Находит лучший доступный образ, определяет GPU
    для режима Webots, затем запускает RunScreen с задачей docker run.
    """
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
        # No image available - offer to run docker-setup to get one.
        # Образ не найден - предлагаем запустить docker-setup чтобы его получить.
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

    # Only detect GPU for Webots - the controller does not need a display.
    # GPU определяем только для Webots - контроллеру дисплей не нужен.
    gpu = _detect_gpu() if mode == "webots" else "software"

    label = "Webots simulator" if mode == "webots" else "Controller"
    _RunApp(
        f"Running {label} — Docker",
        lambda s: _task_run_docker(s, image, mode, gpu),
    ).run()


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
        # No mode given - ask the user how they want to run.
        # Режим не указан - спрашиваем пользователя как он хочет запустить.
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
