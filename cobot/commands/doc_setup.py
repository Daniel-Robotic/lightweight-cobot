from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

from cobot import process, ui
from cobot.ui import done
from cobot.process import StepProgress

_PROJECT_DIR = Path(__file__).parent.parent.parent

# The documentation source lives inside the project; it is mounted into the container
# so MkDocs picks up live edits without rebuilding the image.
# Исходники документации находятся внутри проекта; директория монтируется в контейнер,
# чтобы MkDocs подхватывал изменения вживую без пересборки образа.
_DOC_DIR = _PROJECT_DIR / "doc" / "lwc-doc"
_IMAGE_NAME = "lwc-docs"
_CONTAINER_NAME = "lwc-docs"
_DEFAULT_PORT = "8000"


def _docker(*args: str, capture: bool = False) -> subprocess.CompletedProcess:
    """Run a docker subcommand.
    Запускает подкоманду docker.
    """
    return subprocess.run(["docker", *args], capture_output=capture, text=True)


def _is_running() -> bool:
    """Return True if the lwc-docs container is currently running.
    Возвращает True если контейнер lwc-docs запущен.
    """
    r = _docker("ps", "--filter", f"name={_CONTAINER_NAME}", "--format", "{{.Names}}", capture=True)
    return _CONTAINER_NAME in r.stdout


def _image_exists() -> bool:
    """Return True if the lwc-docs Docker image exists locally.
    Возвращает True если Docker-образ lwc-docs существует локально.
    """
    return bool(_docker("images", "-q", _IMAGE_NAME, capture=True).stdout.strip())


def _build_docs_image(p: StepProgress, lo: float, hi: float) -> bool:
    """Build the lwc-docs image, mapping "Step X/Y" to the lo..hi progress slice.
    Собирает образ lwc-docs, отображая "Step X/Y" на участок lo..hi прогресса.
    """
    p.raw("[cyan]▸[/cyan] Сборка образа документации (один раз)...")
    env = {**os.environ, "DOCKER_BUILDKIT": "0"}

    def on_line(s: str) -> None:
        if s:
            p.log(s)
        m = re.match(r"Step (\d+)/(\d+) :", s)
        if m:
            step, total = int(m.group(1)), int(m.group(2))
            p.set(lo + step / total * (hi - lo), f"шаг {step}/{total}")

    rc = process.stream(["docker", "build", "-t", _IMAGE_NAME, str(_DOC_DIR)],
                        env=env, on_line=on_line)
    if rc != 0:
        p.raw("[red]Сборка образа не удалась.[/red]")
        return False
    p.raw("[green]✓[/green] Образ документации готов")
    return True


def _start_container(p: StepProgress, port: str) -> bool:
    """Start the MkDocs container on the given port. Returns True on success.
    Запускает контейнер MkDocs на заданном порту. Возвращает True при успехе.
    """
    p.set(90, "Запуск сервера MkDocs...")
    p.raw("[cyan]▸[/cyan] Запуск сервера MkDocs...")
    result = _docker(
        "run", "-d", "--name", _CONTAINER_NAME, "--rm",
        "-p", f"{port}:8000",
        "-v", f"{_DOC_DIR}:/docs",
        _IMAGE_NAME, "serve", "--dev-addr=0.0.0.0:8000",
        "--watch", "/docs", "--livereload",
        capture=True,
    )
    if result.returncode != 0:
        p.raw(f"[red]Не удалось запустить контейнер.[/red]\n{result.stderr}")
        return False
    return True


def _task_up(port: str) -> None:
    """Build the image if missing, then start the docs container.
    Собирает образ если отсутствует, затем запускает контейнер документации.
    """
    if _is_running():
        ui.info(f"[green]Документация уже запущена:[/green] http://localhost:{port}")
        ui.note("Остановить:  cobot doc-setup down")
        return
    if not _DOC_DIR.exists():
        ui.error(f"Директория документации не найдена: {_DOC_DIR}")
        return

    ok, fail_msg = True, ""
    with StepProgress("Сервер документации") as p:
        if not _image_exists():
            p.set(0, "Сборка образа документации...")
            if not _build_docs_image(p, 0, 85):
                ok, fail_msg = False, "Сборка образа не удалась"
        else:
            p.log("Образ документации уже собран, пропускаем.")
        if ok and not _start_container(p, port):
            ok, fail_msg = False, "Не удалось запустить контейнер"
        if ok:
            p.set(100, "Сервер запущен")

    if ok:
        done(True, f"Документация доступна: http://localhost:{port}")
        ui.note("Правьте файлы в doc/lwc-doc/docs/ — перезагрузка автоматическая.")
        ui.note("Остановить:  cobot doc-setup down")
    else:
        done(False, fail_msg)


def _task_down() -> None:
    """Stop the lwc-docs container if it is running.
    Останавливает контейнер lwc-docs если он запущен.
    """
    if not _is_running():
        ui.info("[yellow]Контейнер документации не запущен.[/yellow]")
        return
    with StepProgress("Сервер документации") as p:
        p.set(30, "Остановка контейнера...")
        p.raw("[cyan]▸[/cyan] Остановка сервера документации...")
        _docker("stop", _CONTAINER_NAME)
        p.set(100, "Готово")
    done(True, "Контейнер остановлен")


def _task_rebuild(port: str) -> None:
    """Stop the container, remove the old image, rebuild it, and start a fresh container.
    Останавливает контейнер, удаляет старый образ, пересобирает и запускает новый контейнер.
    """
    ok, fail_msg = True, ""
    with StepProgress("Сервер документации — пересборка") as p:
        if _is_running():
            p.set(5, "Остановка контейнера...")
            _docker("stop", _CONTAINER_NAME)
            p.raw("[green]✓[/green] Остановлен.")
        if _image_exists():
            p.set(15, "Удаление старого образа...")
            _docker("rmi", "-f", _IMAGE_NAME)
            p.raw("[green]✓[/green] Образ удалён.")
        p.set(20, "Сборка образа документации...")
        if not _build_docs_image(p, 20, 88):
            ok, fail_msg = False, "Сборка образа не удалась"
        if ok and not _start_container(p, port):
            ok, fail_msg = False, "Не удалось запустить контейнер"
        if ok:
            p.set(100, "Сервер запущен")

    if ok:
        done(True, f"Документация доступна: http://localhost:{port}")
        ui.note("Остановить:  cobot doc-setup down")
    else:
        done(False, fail_msg)


def _normalize_port(value: str) -> str:
    """Return a numeric port string, falling back to the default when invalid.
    Возвращает числовой порт, откатываясь на значение по умолчанию при ошибке.
    """
    value = (value or "").strip()
    return value if value.isdigit() else _DEFAULT_PORT


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
        ui.error("Docker не установлен или отсутствует в PATH.")
        sys.exit(1)

    action = getattr(args, "action", "up")

    if action == "down":
        _task_down()
        return

    port_v = ui.text("Порт для сервера документации:", _DEFAULT_PORT)
    if port_v is None:
        return
    port = _normalize_port(port_v)

    if action == "rebuild":
        _task_rebuild(port)
    else:
        _task_up(port)
