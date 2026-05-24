from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import List, Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, LogScreen, MultiPickScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent

_DIR_OPTIONS = ["build/", "install/", "log/"]
_DIR_MAP = {
    "build/":   _PROJECT_DIR / "build",
    "install/": _PROJECT_DIR / "install",
    "log/":     _PROJECT_DIR / "log",
}


def _task_clean(screen: LogScreen, dirs: List[str]) -> None:
    """Delete the selected top-level directories.
    Удаляет выбранные директории верхнего уровня.
    """
    try:
        screen.write("[bold]Cleaning build artifacts[/bold]\n")
        total = len(dirs)
        for i, label in enumerate(dirs):
            if screen.is_stopped():
                return
            screen.set_progress(i / total * 100, f"Removing {label}...")
            path = _DIR_MAP[label]
            if path.exists():
                shutil.rmtree(path)
                screen.write(f"[green][ok][/green] Removed  {label}")
            else:
                screen.write(f"[dim]Not found: {label}[/dim]")

        if not screen.is_stopped():
            screen.set_progress(100, "Done")
            screen.write("\n[green]Done.[/green]")
            screen.finish(True)

    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


class _CleanApp(App[None]):
    """Clean wizard: lets the user pick which directories to delete, then removes them.
    Мастер очистки: позволяет выбрать директории для удаления, затем удаляет их.
    """

    CSS = SCREEN_CSS

    def __init__(self, all_dirs: bool):
        super().__init__()
        # True = skip the question and delete everything right away.
        # True = пропустить вопрос и сразу удалить всё.
        self._all_dirs = all_dirs

    def on_mount(self) -> None:
        if self._all_dirs:
            self._start(_DIR_OPTIONS)
        else:
            self._ask_dirs()

    def _ask_dirs(self) -> None:
        self.push_screen(
            MultiPickScreen(
                "clean",
                "Which directories to delete?",
                _DIR_OPTIONS,
                note="Space — toggle  ·  Enter — confirm",
            ),
            self._got_dirs,
        )

    def _got_dirs(self, dirs: Optional[List[str]]) -> None:
        if not dirs:
            self.exit()
            return
        self._start(dirs)

    def _start(self, dirs: List[str]) -> None:
        self.push_screen(
            LogScreen(
                "Cleaning",
                lambda s: _task_clean(s, dirs),
                show_progress=True,
            ),
            lambda _: self.exit(),
        )


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
    _CleanApp(all_dirs=(getattr(args, "target", None) == "all")).run()
