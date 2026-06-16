from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path
from typing import Callable

from cobot import ui
from cobot import privilege
from cobot.ui import done
from cobot.process import StepProgress

_PROJECT_DIR = Path(__file__).parent.parent.parent

Log = Callable[[str], None]


def _stop_docker_containers(log: Log) -> None:
    """Stop and force-remove all Docker containers whose name contains "lwc".
    Останавливает и принудительно удаляет все Docker-контейнеры с "lwc" в имени.
    """
    log("[cyan]▸[/cyan] Остановка Docker-контейнеров...")
    result = subprocess.run(
        ["docker", "ps", "-a", "--filter", "name=lwc", "--format", "{{.Names}}"],
        capture_output=True, text=True,
    )
    containers = [c for c in result.stdout.strip().splitlines() if c]
    if not containers:
        log("[dim]Контейнеры проекта не найдены.[/dim]")
        return
    for name in containers:
        subprocess.run(["docker", "rm", "-f", name], capture_output=True)
        log(f"[green]✓[/green] Удалён контейнер: {name}")


def _remove_docker_images(log: Log) -> None:
    """Force-remove all local Docker images whose name or tag contains "lwc".
    Принудительно удаляет все локальные Docker-образы с "lwc" в имени или теге.
    """
    log("[cyan]▸[/cyan] Удаление Docker-образов...")
    result = subprocess.run(
        ["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
        capture_output=True, text=True,
    )
    project_images = [img for img in result.stdout.strip().splitlines() if "lwc" in img.lower()]
    if not project_images:
        log("[dim]Образы проекта не найдены.[/dim]")
        return
    for img in project_images:
        subprocess.run(["docker", "rmi", "-f", img], capture_output=True)
        log(f"[green]✓[/green] Удалён образ: {img}")


def _remove_webots_volume(log: Log) -> None:
    """Remove the lwc-webots-cache Docker volume if it exists.
    Удаляет Docker volume lwc-webots-cache если он существует.
    """
    result = subprocess.run(["docker", "volume", "inspect", "lwc-webots-cache"], capture_output=True)
    if result.returncode != 0:
        log("[dim]Volume кэша Webots не найден, пропускаем.[/dim]")
        return
    subprocess.run(["docker", "volume", "rm", "lwc-webots-cache"], capture_output=True)
    log("[green]✓[/green] Удалён Docker volume: lwc-webots-cache")


def _remove_ros2(log: Log) -> None:
    """Remove all ros-jazzy-* packages, the ros2-apt-source, and the ROS2 source line.
    Удаляет все пакеты ros-jazzy-*, ros2-apt-source и строку source ROS2 из конфигов.
    """
    log("[cyan]▸[/cyan] Удаление пакетов ROS2 Jazzy...")
    if not Path("/opt/ros/jazzy").exists():
        log("[dim]ROS2 Jazzy не найден, пропускаем.[/dim]")
    else:
        subprocess.run(privilege.sudo(["apt", "remove", "-y", "~nros-jazzy-*"]), capture_output=True)
        subprocess.run(privilege.sudo(["apt", "autoremove", "-y"]), capture_output=True)
        log("[green]✓[/green] Пакеты ROS2 Jazzy удалены")
        subprocess.run(privilege.sudo(["apt", "remove", "-y", "ros2-apt-source"]), capture_output=True)
        subprocess.run(privilege.sudo(["apt", "update", "-qq"]), capture_output=True)
        subprocess.run(privilege.sudo(["apt", "autoremove", "-y"]), capture_output=True)
        log("[green]✓[/green] apt-репозиторий ROS2 удалён")

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
        log(f"[green]✓[/green] Очищен ~/{rc_name}")


def _remove_webots(log: Log) -> None:
    """Remove the webots package and clean WEBOTS_HOME from shell configs.
    Удаляет пакет webots и очищает WEBOTS_HOME из конфигов оболочки.
    """
    log("[cyan]▸[/cyan] Удаление Webots...")
    if not shutil.which("webots"):
        log("[dim]Webots не найден, пропускаем.[/dim]")
        return
    subprocess.run(privilege.sudo(["apt", "remove", "-y", "webots"]), capture_output=True)
    subprocess.run(privilege.sudo(["apt", "autoremove", "-y"]), capture_output=True)
    log("[green]✓[/green] Webots удалён")

    for rc_name in [".bashrc", ".zshrc"]:
        rc = Path.home() / rc_name
        if not rc.exists():
            continue
        content = rc.read_text()
        if "WEBOTS_HOME" not in content:
            continue
        new_content = content.replace("\n# Webots\nexport WEBOTS_HOME=/usr/local/webots\n", "\n")
        new_content = new_content.replace("export WEBOTS_HOME=/usr/local/webots\n", "")
        new_content = new_content.replace("# Webots\n", "")
        if new_content != content:
            rc.write_text(new_content)
            log(f"[green]✓[/green] Очищен WEBOTS_HOME из ~/{rc_name}")


def _uninstall_cobot(log: Log) -> None:
    """Uninstall the lightweight-cobot package from the uv tool store.
    Удаляет пакет lightweight-cobot из хранилища инструментов uv.
    """
    log("[cyan]▸[/cyan] Удаление cobot CLI...")
    result = subprocess.run(["uv", "tool", "uninstall", "lightweight-cobot"],
                            capture_output=True, text=True)
    if result.returncode == 0:
        log("[green]✓[/green] cobot удалён")
    else:
        log(f"[yellow]Предупреждение:[/yellow] {result.stderr.strip() or 'не удалось удалить cobot'}")


def _remove_project_dir(log: Log) -> None:
    """Recursively delete the entire project directory from disk.
    Рекурсивно удаляет всю директорию проекта с диска.
    """
    log("[cyan]▸[/cyan] Удаление директории проекта...")
    shutil.rmtree(_PROJECT_DIR)
    log(f"[green]✓[/green] Удалено {_PROJECT_DIR}")


def _delete(remove_ros: bool, remove_webots: bool) -> None:
    """Run all deletion steps in order, with progress split across the active steps.
    Выполняет все шаги удаления по порядку, распределяя прогресс между активными шагами.
    """
    ok, fail_msg = True, ""
    with StepProgress("Удаление проекта") as p:
        try:
            p.set(0, "Остановка контейнеров...")
            _stop_docker_containers(p.raw)
            _remove_webots_volume(p.raw)

            p.set(20, "Удаление Docker-образов...")
            _remove_docker_images(p.raw)

            pct = 40
            if remove_ros:
                p.set(pct, "Удаление ROS2 Jazzy...")
                _remove_ros2(p.raw)
                pct = 65
            if remove_webots:
                p.set(pct, "Удаление Webots...")
                _remove_webots(p.raw)
                pct = 75

            p.set(pct, "Удаление cobot CLI...")
            _uninstall_cobot(p.raw)

            p.set(88, "Удаление директории проекта...")
            _remove_project_dir(p.raw)
            p.set(100, "Готово")
        except Exception as exc:
            ok, fail_msg = False, str(exc)

    done(ok, "Проект полностью удалён" if ok else fail_msg)


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser(
        "delete",
        help="Remove the project, Docker images, containers, and optionally ROS2",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    ui.header("Удаление проекта", "контейнеры, образы, опционально ROS2/Webots")

    if not ui.confirm(
        "Это безвозвратно удалит проект, Docker-образы и контейнеры. Продолжить?",
        default=False,
    ):
        return

    remove_ros = ui.confirm("Также удалить ROS2 Jazzy из системы?", default=False)

    remove_webots = False
    if shutil.which("webots"):
        remove_webots = ui.confirm("Также удалить Webots из системы?", default=False)

    # apt removals need root — acquire sudo once before starting.
    # Удаление через apt требует root — получаем sudo один раз перед началом.
    if (remove_ros or remove_webots) and not privilege.ensure_sudo():
        return

    _delete(remove_ros, remove_webots)
