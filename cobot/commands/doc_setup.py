from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Callable, Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, InputScreen, LogScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent

# The documentation source lives inside the project. We mount it into the container so
# MkDocs can pick up live edits without rebuilding the image.
# Исходники документации находятся внутри проекта. Монтируем директорию в контейнер, чтобы
# MkDocs мог подхватывать изменения вживую без пересборки образа.
_DOC_DIR = _PROJECT_DIR / "doc" / "lwc-doc"
_IMAGE_NAME = "lwc-docs"
_CONTAINER_NAME = "lwc-docs"
_DEFAULT_PORT = "8000"

Write = Callable[[str], None]


# Thin wrapper around docker so we do not repeat ["docker", ...] everywhere.
# Тонкая обёртка вокруг docker, чтобы не повторять ["docker", ...] везде.
def _docker(*args: str, capture: bool = False) -> subprocess.CompletedProcess:
    """Run a docker subcommand. Pass capture=True to capture stdout/stderr instead of printing.
    Запускает подкоманду docker. capture=True перехватывает stdout/stderr вместо вывода на экран.
    """
    return subprocess.run(["docker", *args], capture_output=capture, text=True)


# Check whether the docs container is currently running.
# Проверяем, запущен ли сейчас контейнер с документацией.
def _is_running() -> bool:
    """Return True if the lwc-docs container is currently running.
    Возвращает True если контейнер lwc-docs в данный момент запущен.
    """
    r = _docker("ps", "--filter", f"name={_CONTAINER_NAME}", "--format", "{{.Names}}", capture=True)
    return _CONTAINER_NAME in r.stdout


# Check whether the docs Docker image has already been built.
# Проверяем, был ли уже собран Docker-образ для документации.
def _image_exists() -> bool:
    """Return True if the lwc-docs Docker image exists locally.
    Возвращает True если Docker-образ lwc-docs существует локально.
    """
    return bool(_docker("images", "-q", _IMAGE_NAME, capture=True).stdout.strip())


# Build the MkDocs Docker image. Only needs to run once.
# Progress comes from parsing "Step X/Y" lines in the docker build output.
# Собираем Docker-образ MkDocs. Нужно сделать только один раз.
# Прогресс получаем, парся строки "Step X/Y" из вывода docker build.
def _build_docs_image(
    write: Write,
    on_progress: Optional[Callable[[float], None]] = None,
    register_proc: Optional[Callable] = None,
) -> bool:
    """Build the lwc-docs Docker image from the doc/lwc-doc directory. Returns True on success.
    Собирает Docker-образ lwc-docs из директории doc/lwc-doc. Возвращает True при успехе.
    """
    write("[cyan][*][/cyan] Building documentation image (runs once)...")
    # DOCKER_BUILDKIT=0 gives us "Step X/Y" lines that we can parse for progress.
    # DOCKER_BUILDKIT=0 даёт нам строки "Step X/Y", которые можно парсить для прогресса.
    env = {**os.environ, "DOCKER_BUILDKIT": "0"}
    proc = subprocess.Popen(
        ["docker", "build", "-t", _IMAGE_NAME, str(_DOC_DIR)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
    )
    if register_proc:
        register_proc(proc)
    for line in proc.stdout:
        s = line.rstrip()
        if s:
            write(s)
        if on_progress:
            m = re.match(r"Step (\d+)/(\d+) :", line)
            if m:
                step, total = int(m.group(1)), int(m.group(2))
                on_progress(step / total * 100)
    proc.wait()
    if proc.returncode in (-9, -15):
        return False
    if proc.returncode == 0:
        write("[green][ok][/green] Documentation image ready")
        return True
    write("[red]Image build failed.[/red]")
    return False


# Start the docs server. Builds the image first if it does not exist yet.
# Запускаем сервер документации. Сначала собирает образ, если он ещё не существует.
def _task_up(screen: LogScreen, port: str) -> None:
    """Worker function for the "up" action. Builds the image if missing, then starts the container.
    Рабочая функция для действия "up". Собирает образ если отсутствует, затем запускает контейнер.
    """
    try:
        if _is_running():
            screen.write(f"[green]Docs already running at:[/green] http://localhost:{port}")
            screen.write("  Stop with: [bold]cobot doc-setup down[/bold]")
            if not screen.is_stopped():
                screen.finish(True)
            return

        if not _DOC_DIR.exists():
            screen.write(f"[red]Doc directory not found:[/red] {_DOC_DIR}")
            if not screen.is_stopped():
                screen.finish(False)
            return

        if not _image_exists():
            screen.set_progress(0, "Building documentation image...")
            ok = _build_docs_image(
                screen.write,
                on_progress=lambda p: screen.set_progress(p * 0.85, "Building documentation image..."),
                register_proc=screen.set_proc,
            )
            if screen.is_stopped():
                return
            if not ok:
                screen.finish(False)
                return
        else:
            screen.write("[dim]Documentation image already built, skipping.[/dim]")

        if screen.is_stopped():
            return

        screen.set_progress(88, "Starting MkDocs server...")
        screen.write("\n[cyan][*][/cyan] Starting MkDocs server...")
        result = _docker(
            "run", "-d", "--name", _CONTAINER_NAME, "--rm",
            "-p", f"{port}:8000",
            # Mount the docs directory so edits appear live without restarting the container.
            # Монтируем директорию с документацией, чтобы изменения появлялись сразу без перезапуска.
            "-v", f"{_DOC_DIR}:/docs",
            _IMAGE_NAME, "serve", "--dev-addr=0.0.0.0:8000",
            capture=True,
        )
        if screen.is_stopped():
            return
        if result.returncode != 0:
            screen.write(f"[red]Failed to start container.[/red]\n{result.stderr}")
            screen.finish(False)
            return

        screen.set_progress(100, "Server running")
        screen.write(f"\n[green]Docs running at:[/green] http://localhost:{port}")
        screen.write("  Edit files in [bold]doc/lwc-doc/docs/[/bold] — reloads automatically.")
        screen.write("  Stop with: [bold]cobot doc-setup down[/bold]")
        if not screen.is_stopped():
            screen.finish(True)

    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"[red]Error:[/red] {exc}")
            screen.finish(False)


# Stop the running docs container.
# Останавливаем работающий контейнер с документацией.
def _task_down(screen: LogScreen) -> None:
    """Worker function for the "down" action. Stops the lwc-docs container if it is running.
    Рабочая функция для действия "down". Останавливает контейнер lwc-docs если он запущен.
    """
    try:
        if not _is_running():
            screen.write("[yellow]Docs container is not running.[/yellow]")
            if not screen.is_stopped():
                screen.finish(True)
            return
        screen.set_progress(30, "Stopping container...")
        screen.write("[cyan][*][/cyan] Stopping documentation server...")
        _docker("stop", _CONTAINER_NAME)
        if screen.is_stopped():
            return
        screen.set_progress(100, "Done")
        screen.write("[green][ok][/green] Container stopped.")
        screen.finish(True)
    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"[red]Error:[/red] {exc}")
            screen.finish(False)


# Stop the container, remove the old image, rebuild it, and start a new container.
# Останавливаем контейнер, удаляем старый образ, пересобираем и запускаем новый контейнер.
def _task_rebuild(screen: LogScreen, port: str) -> None:
    """Worker function for the "rebuild" action. Stops the container, removes the old image,
    rebuilds it, and starts a fresh container on the given port.
    Рабочая функция для действия "rebuild". Останавливает контейнер, удаляет старый образ,
    пересобирает его и запускает новый контейнер на указанном порту.
    """
    try:
        if _is_running():
            screen.set_progress(5, "Stopping container...")
            screen.write("[cyan][*][/cyan] Stopping existing container...")
            _docker("stop", _CONTAINER_NAME)
            if screen.is_stopped():
                return
            screen.write("[green][ok][/green] Stopped.")

        if _image_exists():
            screen.set_progress(15, "Removing old image...")
            screen.write("[cyan][*][/cyan] Removing old image...")
            _docker("rmi", "-f", _IMAGE_NAME)
            if screen.is_stopped():
                return
            screen.write("[green][ok][/green] Image removed.")

        screen.set_progress(20, "Building documentation image...")
        ok = _build_docs_image(
            screen.write,
            on_progress=lambda p: screen.set_progress(20 + p * 0.68, "Building documentation image..."),
            register_proc=screen.set_proc,
        )
        if screen.is_stopped():
            return
        if not ok:
            screen.finish(False)
            return

        screen.set_progress(90, "Starting MkDocs server...")
        screen.write("\n[cyan][*][/cyan] Starting MkDocs server...")
        result = _docker(
            "run", "-d", "--name", _CONTAINER_NAME, "--rm",
            "-p", f"{port}:8000",
            "-v", f"{_DOC_DIR}:/docs",
            _IMAGE_NAME, "serve", "--dev-addr=0.0.0.0:8000",
            capture=True,
        )
        if screen.is_stopped():
            return
        if result.returncode != 0:
            screen.write(f"[red]Failed to start container.[/red]\n{result.stderr}")
            screen.finish(False)
            return

        screen.set_progress(100, "Server running")
        screen.write(f"\n[green]Docs running at:[/green] http://localhost:{port}")
        screen.write("  Stop with: [bold]cobot doc-setup down[/bold]")
        if not screen.is_stopped():
            screen.finish(True)

    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"[red]Error:[/red] {exc}")
            screen.finish(False)


# One app handles all three actions (up/down/rebuild) by branching in on_mount.
# Одно приложение обрабатывает все три действия (up/down/rebuild), разветвляясь в on_mount.
class _DocApp(App[None]):
    """Documentation server app. Handles "up", "down", and "rebuild" actions by branching
    in on_mount to the appropriate LogScreen task.
    Приложение сервера документации. Обрабатывает действия "up", "down" и "rebuild",
    разветвляясь в on_mount к соответствующей задаче LogScreen.
    """
    CSS = SCREEN_CSS

    def __init__(self, action: str):
        super().__init__()
        self._action = action

    def on_mount(self) -> None:
        if self._action == "down":
            self.push_screen(
                LogScreen("Documentation server", _task_down, show_progress=True),
                lambda _: self.exit(),
            )
        elif self._action == "rebuild":
            self.push_screen(
                InputScreen("Step 1 of 1", "Port to serve documentation on:", _DEFAULT_PORT),
                self._got_port_rebuild,
            )
        else:
            self.push_screen(
                InputScreen("Step 1 of 1", "Port to serve documentation on:", _DEFAULT_PORT),
                self._got_port_up,
            )

    def _got_port_up(self, port: Optional[str]) -> None:
        if port is None:
            self.exit()
            return
        # Use the default port if the user cleared the input or typed something that is not a number.
        # Используем порт по умолчанию если пользователь очистил ввод или написал не число.
        p = (port.strip() or _DEFAULT_PORT) if port.isdigit() or not port.strip() else _DEFAULT_PORT
        self.push_screen(
            LogScreen("Documentation server", lambda s: _task_up(s, p), show_progress=True),
            lambda _: self.exit(),
        )

    def _got_port_rebuild(self, port: Optional[str]) -> None:
        if port is None:
            self.exit()
            return
        p = (port.strip() or _DEFAULT_PORT) if port.isdigit() or not port.strip() else _DEFAULT_PORT
        self.push_screen(
            LogScreen("Documentation server — rebuild", lambda s: _task_rebuild(s, p), show_progress=True),
            lambda _: self.exit(),
        )


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("doc-setup", help="Deploy or stop the documentation server")
    p.add_argument(
        "action",
        nargs="?",
        choices=["up", "down", "rebuild"],
        default="up",
        help="up — start (default), down — stop, rebuild — rebuild image and restart",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    if not shutil.which("docker"):
        from rich.console import Console
        Console().print("[red]Error:[/red] Docker is not installed or not on PATH.")
        sys.exit(1)

    action = getattr(args, "action", "up")
    _DocApp(action).run()
