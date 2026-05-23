from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, InputScreen, LogScreen, PickScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent
_BUILD_DIR = _PROJECT_DIR / "build"
_INSTALL_DIR = _PROJECT_DIR / "install"
_LOG_DIR = _PROJECT_DIR / "log"


def _remove_dir(path: Path, write) -> None:
    """Remove a directory tree if it exists, logging the result.
    Рекурсивно удаляет директорию если она существует, выводя результат в лог.
    """
    if path.exists():
        shutil.rmtree(path)
        write(f"[green][ok][/green] Removed  {path.relative_to(_PROJECT_DIR)}")
    else:
        write(f"[dim]Not found: {path.relative_to(_PROJECT_DIR)}[/dim]")


def _task_clean_all(screen: LogScreen) -> None:
    """Delete build/, install/, and log/ directories entirely.
    Полностью удаляет директории build/, install/ и log/.
    """
    try:
        screen.set_progress(0, "Cleaning build/...")
        screen.write("[bold]Cleaning all build artifacts[/bold]\n")

        _remove_dir(_BUILD_DIR, screen.write)
        if screen.is_stopped():
            return

        screen.set_progress(40, "Cleaning install/...")
        _remove_dir(_INSTALL_DIR, screen.write)
        if screen.is_stopped():
            return

        screen.set_progress(75, "Cleaning log/...")
        _remove_dir(_LOG_DIR, screen.write)
        if screen.is_stopped():
            return

        screen.set_progress(100, "Done")
        screen.write("\n[green]All build artifacts removed.[/green]")
        screen.finish(True)

    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


def _task_clean_package(screen: LogScreen, package: str) -> None:
    """Delete build/<pkg>, install/<pkg>, and all log/*/<pkg> directories.
    Удаляет build/<pkg>, install/<pkg> и все log/*/<pkg> директории.
    """
    try:
        screen.set_progress(0, f"Cleaning {package}...")
        screen.write(f"[bold]Cleaning package: {package}[/bold]\n")

        _remove_dir(_BUILD_DIR / package, screen.write)
        if screen.is_stopped():
            return

        screen.set_progress(40, f"Cleaning install/{package}...")
        _remove_dir(_INSTALL_DIR / package, screen.write)
        if screen.is_stopped():
            return

        screen.set_progress(70, f"Cleaning log entries for {package}...")
        # colcon creates one subdirectory per package inside each timestamped log run.
        # colcon создаёт по одной поддиректории на пакет внутри каждого лога с временной меткой.
        if _LOG_DIR.exists():
            removed = 0
            for log_run in _LOG_DIR.iterdir():
                if not log_run.is_dir():
                    continue
                pkg_log = log_run / package
                if pkg_log.exists():
                    shutil.rmtree(pkg_log)
                    screen.write(
                        f"[green][ok][/green] Removed  log/{log_run.name}/{package}"
                    )
                    removed += 1
            if removed == 0:
                screen.write(f"[dim]No log entries found for {package}[/dim]")
        else:
            screen.write("[dim]Not found: log/[/dim]")

        if screen.is_stopped():
            return

        screen.set_progress(100, "Done")
        screen.write(f"\n[green]Package '{package}' artifacts removed.[/green]")
        screen.finish(True)

    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


class _CleanApp(App[None]):
    """Clean wizard: asks what to clean (all / specific package) then runs deletion.
    Мастер очистки: спрашивает что удалить (всё / конкретный пакет), затем выполняет удаление.
    """

    CSS = SCREEN_CSS

    def __init__(self, target: Optional[str]):
        super().__init__()
        # None = ask, "all" = delete everything, anything else = package name.
        # None = спросить, "all" = удалить всё, иначе = имя пакета.
        self._target = target

    def on_mount(self) -> None:
        if self._target is None:
            self._ask_target()
        elif self._target == "all":
            self._start_all()
        else:
            self._start_package(self._target)

    def _ask_target(self) -> None:
        self.push_screen(
            PickScreen(
                "clean",
                "What do you want to clean?",
                ["All (build/ install/ log/)", "Specific package"],
                "All (build/ install/ log/)",
            ),
            self._got_target_choice,
        )

    def _got_target_choice(self, value: Optional[str]) -> None:
        if value is None:
            self.exit()
            return
        if value.startswith("All"):
            self._start_all()
        else:
            self._ask_package()

    def _ask_package(self) -> None:
        self.push_screen(
            InputScreen(
                "clean",
                "Package name to clean:",
                "",
                note="Example: iiwa_controller",
            ),
            self._got_package,
        )

    def _got_package(self, value: Optional[str]) -> None:
        if value is None or not value.strip():
            self.exit()
            return
        self._start_package(value.strip())

    def _start_all(self) -> None:
        self.push_screen(
            LogScreen("Cleaning all artifacts", _task_clean_all, show_progress=True),
            lambda _: self.exit(),
        )

    def _start_package(self, package: str) -> None:
        self.push_screen(
            LogScreen(
                f"Cleaning package: {package}",
                lambda s: _task_clean_package(s, package),
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
        metavar="all | PACKAGE",
        default=None,
        help="'all' to remove everything, or a package name to clean only that package",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    """Entry point for the clean command.
    Точка входа для команды clean.
    """
    _CleanApp(getattr(args, "target", None)).run()
