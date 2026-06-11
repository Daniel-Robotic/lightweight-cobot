from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import List

from cobot import ui
from cobot.ui import header, done

_PROJECT_DIR = Path(__file__).parent.parent.parent

_DIR_OPTIONS = ["build/", "install/", "log/"]
_DIR_MAP = {
    "build/":   _PROJECT_DIR / "build",
    "install/": _PROJECT_DIR / "install",
    "log/":     _PROJECT_DIR / "log",
}


def _clean(dirs: List[str]) -> None:
    """Delete the selected top-level directories, printing the outcome of each.
    Удаляет выбранные директории верхнего уровня, печатая результат по каждой.
    """
    header("Очистка артефактов сборки")
    removed = False
    for label in dirs:
        path = _DIR_MAP[label]
        if path.exists():
            shutil.rmtree(path)
            ui.info(f"  [green]✓[/green] Удалено  {label}")
            removed = True
        else:
            ui.info(f"  [dim]Нет:[/dim] {label}")
    done(True, "Очищено" if removed else "Нечего удалять")


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser(
        "clean",
        help="Remove colcon build artifacts (build/ install/ log/)",
    )
    p.add_argument(
        "target",
        nargs="?",
        metavar="all",
        default=None,
        help="'all' to skip the prompt and delete all three directories at once",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    """Entry point for the clean command.
    Точка входа для команды clean.
    """
    if getattr(args, "target", None) == "all":
        _clean(_DIR_OPTIONS)
        return

    dirs = ui.multiselect(
        "Какие директории удалить?",
        _DIR_OPTIONS,
        note="Space — отметить · Enter — подтвердить",
    )
    if not dirs:
        return
    _clean(dirs)
