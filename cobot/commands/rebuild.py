from __future__ import annotations

import argparse
import subprocess
from pathlib import Path
from typing import List, Optional

from cobot import ui, process
from cobot.commands.local_setup import _ros2_env

_PROJECT_DIR = Path(__file__).parent.parent.parent


def _count_packages(packages: List[str], env: dict) -> int:
    """Count how many colcon packages will be built so we can show X / total progress.
    Считает количество пакетов colcon для отображения прогресса X / всего.
    """
    list_cmd = ["colcon", "list", "--base-paths", "src"]
    if packages:
        list_cmd += ["--packages-select"] + packages
    result = subprocess.run(list_cmd, capture_output=True, text=True,
                            cwd=_PROJECT_DIR, env=env)
    return max(len([l for l in result.stdout.splitlines() if l.strip()]), 1)


def _rebuild(packages: List[str], symlink: bool) -> None:
    """Run colcon build for the selected packages (or all), streaming live output
    with a per-package progress bar.
    Запускает colcon build для выбранных пакетов (или всех), транслируя живой вывод
    с прогресс-баром по пакетам.
    """
    env = _ros2_env()
    total = _count_packages(packages, env)

    pkg_label = " ".join(packages) if packages else "все пакеты"
    symlink_label = " --symlink-install" if symlink else ""

    cmd = ["colcon", "build", "--base-paths", "src"]
    if symlink:
        cmd.append("--symlink-install")
    if packages:
        cmd += ["--packages-select"] + packages

    built = 0

    def _parse(line: str):
        nonlocal built
        if "Finished <<<" in line or "Failed <<<" in line:
            built += 1
            return (built / total * 100, f"{built} / {total} пакетов")
        return None

    rc = process.run_step(
        f"colcon build{symlink_label} — {pkg_label}",
        cmd,
        env=env,
        cwd=str(_PROJECT_DIR),
        total=100.0,
        parse_progress=_parse,
        success_msg="Сборка завершена",
        fail_msg="Сборка завершилась с ошибкой",
    )
    if rc in (0, -9, -15):
        ui.note("Активируйте окружение:  source install/setup.bash")


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser(
        "rebuild",
        help="Rebuild ROS2 packages in src/ with colcon",
    )
    p.add_argument(
        "packages",
        nargs="*",
        metavar="PACKAGE",
        help="Packages to rebuild (omit to be asked, or leave empty for all)",
    )
    p.set_defaults(symlink=None)
    p.add_argument(
        "--symlink-install",
        dest="symlink",
        action="store_true",
        help="Pass --symlink-install to colcon build",
    )
    p.add_argument(
        "--no-symlink-install",
        dest="symlink",
        action="store_false",
        help="Do not pass --symlink-install to colcon build",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    """Entry point for the rebuild command.
    Точка входа для команды rebuild.
    """
    packages: Optional[List[str]] = args.packages if args.packages else None

    if packages is None:
        value = ui.text(
            "Какие пакеты пересобрать? (через пробел, пусто — все)",
            "",
            note="Пример: iiwa_controller iiwa_bringup",
        )
        if value is None:
            return
        packages = value.split() if value.strip() else []

    symlink = args.symlink
    if symlink is None:
        choice = ui.select("Использовать --symlink-install?", ["Да", "Нет"], "Да")
        if choice is None:
            return
        symlink = choice == "Да"

    _rebuild(packages, symlink)
