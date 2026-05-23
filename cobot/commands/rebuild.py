from __future__ import annotations

import argparse
import subprocess
from pathlib import Path
from typing import List, Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, InputScreen, LogScreen, PickScreen
from cobot.commands.local_setup import _ros2_env

_PROJECT_DIR = Path(__file__).parent.parent.parent


def _task_rebuild(screen: LogScreen, packages: List[str], symlink: bool) -> None:
    """Run colcon build for the selected packages (or all if packages is empty).
    Streams output to the log and tracks per-package progress.

    Запускает colcon build для выбранных пакетов (или всех если packages пуст).
    Транслирует вывод в лог и отслеживает прогресс по каждому пакету.
    """
    try:
        env = _ros2_env()

        # Count packages so we can show X / total progress.
        # Считаем пакеты чтобы показывать X / всего в прогрессе.
        list_cmd = ["colcon", "list", "--base-paths", "src"]
        if packages:
            list_cmd += ["--packages-select"] + packages
        list_result = subprocess.run(
            list_cmd, capture_output=True, text=True,
            cwd=_PROJECT_DIR, env=env,
        )
        total = max(len([l for l in list_result.stdout.splitlines() if l.strip()]), 1)

        pkg_label = " ".join(packages) if packages else "all packages"
        symlink_label = " --symlink-install" if symlink else ""
        screen.write(f"[bold]colcon build{symlink_label} — {pkg_label}[/bold]\n")
        screen.set_progress(0, f"0 / {total} packages done")
        built = 0

        cmd = ["colcon", "build", "--base-paths", "src"]
        if symlink:
            cmd.append("--symlink-install")
        if packages:
            cmd += ["--packages-select"] + packages

        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, cwd=_PROJECT_DIR, env=env,
        )
        screen.set_proc(proc)

        for line in proc.stdout:
            s = line.rstrip()
            if s:
                screen.write(s)
            if "Finished <<<" in line or "Failed <<<" in line:
                built += 1
                screen.set_progress(
                    built / total * 100,
                    f"{built} / {total} packages done",
                )
        proc.wait()

        if screen.is_stopped():
            return

        if proc.returncode not in (0, -9):
            screen.write("\n[red]Build failed.[/red]")
            screen.finish(False)
            return

        screen.set_progress(100, "Done")
        screen.write("\n[green]Build complete.[/green]")
        screen.finish(True)

    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


class _RebuildApp(App[None]):
    """Rebuild wizard: optionally asks for packages and symlink flag, then runs colcon.
    Мастер пересборки: опционально спрашивает пакеты и флаг symlink, затем запускает colcon.
    """

    CSS = SCREEN_CSS

    def __init__(self, packages: Optional[List[str]], symlink: Optional[bool]):
        super().__init__()
        # None means "ask the user interactively".
        # None означает "спросить пользователя интерактивно".
        self._packages = packages
        self._symlink = symlink

    def on_mount(self) -> None:
        if self._packages is None:
            self._ask_packages()
        elif self._symlink is None:
            self._ask_symlink()
        else:
            self._start()

    def _ask_packages(self) -> None:
        self.push_screen(
            InputScreen(
                "rebuild",
                "Packages to rebuild (space-separated, leave empty for all):",
                "",
                note="Example: iiwa_controller iiwa_bringup",
            ),
            self._got_packages,
        )

    def _got_packages(self, value: Optional[str]) -> None:
        if value is None:
            self.exit()
            return
        self._packages = value.split() if value.strip() else []
        if self._symlink is None:
            self._ask_symlink()
        else:
            self._start()

    def _ask_symlink(self) -> None:
        self.push_screen(
            PickScreen(
                "rebuild",
                "Use --symlink-install?",
                ["Yes", "No"],
                "Yes",
            ),
            self._got_symlink,
        )

    def _got_symlink(self, value: Optional[str]) -> None:
        if value is None:
            self.exit()
            return
        self._symlink = value == "Yes"
        self._start()

    def _start(self) -> None:
        self.push_screen(
            LogScreen(
                "Rebuilding packages",
                lambda s: _task_rebuild(s, self._packages, self._symlink),
                show_progress=True,
            ),
            lambda _: self.exit(),
        )


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
    # Convert empty list to None so the TUI asks interactively.
    # Преобразуем пустой список в None, чтобы TUI спросил интерактивно.
    packages = args.packages if args.packages else None
    _RebuildApp(packages, args.symlink).run()
