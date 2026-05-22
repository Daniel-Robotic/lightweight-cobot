from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

from textual.app import App

from cobot.tui import SCREEN_CSS, LogScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent


# Pull the latest commits from the remote and reinstall the cobot CLI in one go.
# Progress bar: fetch (0-30%), pull (30-80%), reinstall (80-100%).
# Скачиваем последние коммиты с удалённого репозитория и переустанавливаем cobot CLI за один раз.
# Прогресс-бар: fetch (0-30%), pull (30-80%), переустановка (80-100%).
def _task_update(screen: LogScreen) -> None:
    try:
        # Find out which branch we are on so we can fetch and pull the right one.
        # Определяем на какой ветке мы находимся, чтобы делать fetch и pull нужной ветки.
        branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=_PROJECT_DIR, text=True,
        ).strip()
        screen.write(f"[cyan][*][/cyan] Branch: [bold]{branch}[/bold]")

        # Fetch (0 → 30 %)
        screen.set_progress(0, "Fetching from remote...")
        screen.write("[cyan][*][/cyan] Fetching from remote...")
        fetch = subprocess.run(
            ["git", "fetch", "origin"],
            cwd=_PROJECT_DIR, capture_output=True, text=True,
        )
        if screen.is_stopped():
            return
        if fetch.returncode != 0:
            screen.write(f"[red]Fetch failed:[/red] {fetch.stderr.strip()}")
            screen.finish(False)
            return
        screen.set_progress(30)

        # Count how many commits the remote is ahead of us.
        # Считаем сколько коммитов нас опережает удалённый репозиторий.
        behind = subprocess.check_output(
            ["git", "rev-list", f"HEAD..origin/{branch}", "--count"],
            cwd=_PROJECT_DIR, text=True,
        ).strip()

        if behind == "0":
            if not screen.is_stopped():
                screen.set_progress(100, "Already up to date")
                screen.write("[green][ok][/green] Already up to date.")
                screen.finish(True)
            return

        # Show which commits are coming in so the user knows what changed.
        # Показываем какие коммиты приходят, чтобы пользователь знал что изменилось.
        screen.write(f"\n[bold]{behind} new commit(s):[/bold]")
        log_lines = subprocess.check_output(
            ["git", "log", f"HEAD..origin/{branch}", "--oneline"],
            cwd=_PROJECT_DIR, text=True,
        ).strip().splitlines()
        for line in log_lines:
            screen.write(f"  [dim]{line}[/dim]")

        # Pull (30 → 80 %)
        screen.set_progress(30, "Pulling changes...")
        screen.write("\n[cyan][*][/cyan] Pulling changes...")
        pull = subprocess.run(
            ["git", "pull", "origin", branch],
            capture_output=True, text=True, cwd=_PROJECT_DIR,
        )
        if screen.is_stopped():
            return
        if pull.returncode != 0:
            for line in (pull.stdout + pull.stderr).splitlines():
                if line.strip():
                    screen.write(line)
            screen.write("[red]Pull failed.[/red]")
            screen.finish(False)
            return
        screen.set_progress(80)

        # Reinstall (80 → 100 %)
        # Reinstall so the cobot binary picks up any new dependencies from pyproject.toml.
        # Переустанавливаем, чтобы бинарник cobot подхватил новые зависимости из pyproject.toml.
        screen.set_progress(80, "Reinstalling cobot CLI...")
        screen.write("\n[cyan][*][/cyan] Reinstalling cobot CLI...")
        reinstall = subprocess.run(
            ["uv", "tool", "install", "--editable", str(_PROJECT_DIR)],
            capture_output=True, text=True,
        )
        if screen.is_stopped():
            return
        if reinstall.returncode == 0:
            screen.write("[green][ok][/green] cobot reinstalled")
        else:
            screen.write(f"[yellow]Warning:[/yellow] reinstall failed — {reinstall.stderr.strip()}")

        if not screen.is_stopped():
            screen.set_progress(100, "Done")
            screen.write("\n[green]Project updated successfully.[/green]")
            screen.finish(True)

    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


class _UpdateApp(App[None]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            LogScreen("Updating project", _task_update, show_progress=True),
            lambda _: self.exit(),
        )


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("update", help="Pull latest changes from the remote git branch")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    _UpdateApp().run()
