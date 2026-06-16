from __future__ import annotations

import argparse
import os
import shutil
import socket
import subprocess
from pathlib import Path
from typing import List, Optional

from cobot import process, ui
from cobot import privilege
from cobot.ui import done, header
from cobot.commands.local_setup import (
    _WEBOTS_VERSION,
    build_workspace,
    install_webots,
    webots_installed,
)

_PROJECT_DIR = Path(__file__).parent.parent.parent
_CONFIG_PATH = _PROJECT_DIR / "cobot-setting.yaml"
_INSTALL_DIR = _PROJECT_DIR / "install"
_JAZZY_DIR = Path("/opt/ros/jazzy")
# Default Webots installation path for the official .deb package.
# Путь установки Webots по умолчанию для официального .deb-пакета.
_WEBOTS_DEFAULT_HOME = Path("/usr/local/webots")

# Path where the config file is mounted inside the Docker container.
# Путь по которому конфиг-файл монтируется внутри Docker-контейнера.
_CONFIG_IN_CONTAINER = "/ros2_ws/cobot-setting.yaml"

# Container names used for docker run and docker kill.
# Имена контейнеров, используемые для docker run и docker kill.
_CONTAINER_CONTROLLER = "lwc-controller"
_CONTAINER_WEBOTS = "lwc-webots"

# Named Docker volume that stores the Webots asset cache between container runs.
# Именованный Docker volume для хранения кэша ассетов Webots между запусками контейнера.
_WEBOTS_CACHE_VOLUME = "lwc-webots-cache"

# Candidates checked in order - for the controller the webots image is a valid fallback.
# Кандидаты проверяются по порядку - для контроллера образ webots является допустимым запасным.
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


def _detect_webots_home() -> str:
    """Return the WEBOTS_HOME path for the locally installed Linux Webots, or "".
    Возвращает путь WEBOTS_HOME для локально установленного Linux Webots, или "".
    """
    def _is_linux_webots(home: str) -> bool:
        return (Path(home) / "webots").is_file()

    if "WEBOTS_HOME" in os.environ:
        home = os.environ["WEBOTS_HOME"]
        return home if _is_linux_webots(home) else ""
    if _WEBOTS_DEFAULT_HOME.is_dir() and _is_linux_webots(str(_WEBOTS_DEFAULT_HOME)):
        return str(_WEBOTS_DEFAULT_HOME)
    webots_bin = shutil.which("webots")
    if webots_bin:
        home = str(Path(webots_bin).resolve().parent)
        if _is_linux_webots(home):
            return home
    return ""


def _detect_gpu() -> str:
    """Return "nvidia", "mesa", or "software" based on available GPU drivers.
    Возвращает "nvidia", "mesa" или "software" в зависимости от доступных драйверов GPU.
    """
    if shutil.which("nvidia-smi"):
        if subprocess.run(["nvidia-smi"], capture_output=True).returncode == 0:
            return "nvidia"
    if Path("/dev/dri").exists():
        return "mesa"
    return "software"


def _docker_images() -> set:
    """Return the set of "repository:tag" strings for all local Docker images.
    Возвращает множество строк "репозиторий:тег" для всех локальных Docker-образов.
    """
    r = subprocess.run(
        ["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
        capture_output=True, text=True,
    )
    return set(r.stdout.strip().splitlines())


def _find_image(candidates: List[str]) -> Optional[str]:
    """Return the first candidate image that exists locally, or None.
    Возвращает первый образ-кандидат, присутствующий локально, или None.
    """
    available = _docker_images()
    for img in candidates:
        if img in available:
            return img
    return None


# Local (non-Docker) launch
# Локальный (не Docker) запуск
def _run_local(mode: str) -> None:
    """Launch iiwa.launch.py natively. The whole ros2 launch tree runs in its own
    session so a single Ctrl-C tears down every node cleanly.
    Запускает iiwa.launch.py нативно. Всё дерево ros2 launch работает в своей сессии,
    поэтому один Ctrl-C аккуратно завершает каждый узел.
    """
    config = str(_CONFIG_PATH)
    ros_cmd = f"ros2 launch iiwa_bringup iiwa.launch.py setting:={config}"
    if mode == "webots":
        ros_cmd += " simulate:=1"

    webots_home = _detect_webots_home() if mode == "webots" else ""
    webots_export = f"export WEBOTS_HOME={webots_home} && " if webots_home else ""

    full_cmd = (
        f"{webots_export}"
        f"source {_JAZZY_DIR}/setup.bash && "
        f"source {_INSTALL_DIR}/setup.bash && "
        f"{ros_cmd}"
    )

    label = "симулятор Webots" if mode == "webots" else "контроллер"
    header(f"Запуск: {label} (локально)")
    ui.note(ros_cmd)
    if webots_home:
        ui.note(f"WEBOTS_HOME: {webots_home}")
    ui.note("Нажмите Ctrl-C чтобы остановить")

    rc = process.stream(
        ["bash", "-c", full_cmd],
        cwd=str(_PROJECT_DIR),
        new_session=True,
    )
    done(rc in (0, -2, -15, 130), "Остановлено")


# Docker launch
# Запуск в Docker
def _run_docker(image: str, mode: str, gpu: str) -> None:
    """Launch iiwa.launch.py inside a Docker container, forwarding X11/GPU for Webots.
    The container is stopped with ``docker kill`` on Ctrl-C.
    Запускает iiwa.launch.py внутри Docker-контейнера, пробрасывая X11/GPU для Webots.
    Контейнер останавливается через ``docker kill`` по Ctrl-C.
    """
    container = _CONTAINER_WEBOTS if mode == "webots" else _CONTAINER_CONTROLLER

    ros_cmd = (
        "source /ros2_ws/install/setup.bash && "
        f"ros2 launch iiwa_bringup iiwa.launch.py setting:={_CONFIG_IN_CONTAINER}"
    )
    if mode == "webots":
        ros_cmd += " simulate:=1"

    # Remove any stale container with the same name from a previous run.
    # Удаляем устаревший контейнер с таким же именем от предыдущего запуска.
    subprocess.run(["docker", "rm", "-f", container], capture_output=True)

    cmd = [
        "docker", "run", "--rm",
        "--name", container,
        "--network", "host",
        "--hostname", socket.gethostname(),
        "-e", "USER=root",
    ]

    if mode == "webots":
        subprocess.run(["xhost", "+local:docker"], capture_output=True)
        cmd += [
            "-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
            "-e", "QT_X11_NO_MITSHM=1",
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
            "-v", f"{_WEBOTS_CACHE_VOLUME}:/root/.cache/Cyberbotics/Webots",
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
        "software": "Программный рендеринг (llvmpipe)",
    }
    label = "симулятор Webots" if mode == "webots" else "контроллер"
    header(f"Запуск: {label} в Docker")
    ui.note(f"Образ: {image}")
    if mode == "webots":
        ui.note(f"GPU:   {_GPU_LABELS.get(gpu, gpu)}")
    ui.note("Нажмите Ctrl-C чтобы остановить")

    rc = process.stream(
        cmd,
        kill_fn=lambda: subprocess.run(["docker", "kill", container], capture_output=True),
    )
    done(rc in (0, -2, -15, 130), "Остановлено")


def _local_flow(args: argparse.Namespace) -> None:
    """Interactive flow for local launch: check Webots/ROS2/build, then run.
    Интерактивный сценарий локального запуска: проверка Webots/ROS2/сборки, затем запуск.
    """
    mode_v = ui.select(
        "Что запустить?",
        ["Контроллер", "Симулятор Webots"],
        "Контроллер",
    )
    if mode_v is None:
        return
    mode = "webots" if mode_v == "Симулятор Webots" else "controller"

    if mode == "webots" and not webots_installed():
        if not ui.confirm(f"Webots {_WEBOTS_VERSION} не установлен. Установить сейчас?",
                          default=True):
            return
        if not privilege.ensure_sudo() or not install_webots():
            return

    if not _JAZZY_DIR.is_dir():
        if ui.confirm("ROS2 Jazzy не установлен. Запустить local-setup?", default=True):
            from cobot.commands.local_setup import run as _local_setup
            _local_setup(args)
        return

    if not (_INSTALL_DIR / "setup.bash").exists():
        if not ui.confirm("Проект ещё не собран. Собрать сейчас?", default=True):
            return
        if not privilege.ensure_sudo() or not build_workspace():
            return

    _run_local(mode)


def _docker_flow(args: argparse.Namespace) -> None:
    """Interactive flow for Docker launch: pick an image, detect GPU, then run.
    Интерактивный сценарий запуска в Docker: выбор образа, определение GPU, затем запуск.
    """
    if not shutil.which("docker"):
        ui.error("Docker не установлен или отсутствует в PATH.")
        return

    mode_v = ui.select(
        "Что запустить?",
        ["Контроллер", "Симулятор Webots"],
        "Контроллер",
    )
    if mode_v is None:
        return
    mode = "webots" if mode_v == "Симулятор Webots" else "controller"

    candidates = _WEBOTS_IMAGES if mode == "webots" else _CONTROLLER_IMAGES
    image = _find_image(candidates)

    if image is None:
        what = "Webots" if mode == "webots" else "контроллера или Webots"
        if ui.confirm(f"Docker-образ для {what} не найден. Запустить docker-setup?",
                      default=True):
            from cobot.commands.docker_setup import run as _docker_setup
            _docker_setup(args)
        return

    gpu = _detect_gpu() if mode == "webots" else "software"
    _run_docker(image, mode, gpu)


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
        v = ui.select(
            "Как запустить проект?",
            ["Локально (нативный ROS2)", "Docker"],
            "Локально (нативный ROS2)",
        )
        if v is None:
            return
        if v.startswith("Локально"):
            _local_flow(args)
        else:
            _docker_flow(args)
