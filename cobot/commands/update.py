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
    """Worker function that runs inside LogScreen. Fetches the current branch, shows incoming
    commits, pulls changes, then reinstalls the cobot CLI via uv tool install --editable.
    Рабочая функция, выполняемая внутри LogScreen. Получает текущую ветку, показывает входящие
    коммиты, вытягивает изменения, затем переустанавливает cobot CLI через uv tool install --editable.
    """
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
        fetch_proc = subprocess.Popen(
            ["git", "fetch", "origin"],
            cwd=_PROJECT_DIR, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        screen.set_proc(fetch_proc)
        fetch_out, fetch_err = fetch_proc.communicate()
        if screen.is_stopped():
            return
        if fetch_proc.returncode not in (0, -9):
            screen.write(f"[red]Fetch failed:[/red] {fetch_err.strip()}")
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
        pull_proc = subprocess.Popen(
            ["git", "pull", "origin", branch],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, cwd=_PROJECT_DIR,
        )
        screen.set_proc(pull_proc)
        pull_out, pull_err = pull_proc.communicate()
        if screen.is_stopped():
            return
        if pull_proc.returncode not in (0, -9):
            for line in (pull_out + pull_err).splitlines():
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
        reinstall_proc = subprocess.Popen(
            ["uv", "tool", "install", "--editable", str(_PROJECT_DIR)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        screen.set_proc(reinstall_proc)
        reinstall_out, reinstall_err = reinstall_proc.communicate()
        if screen.is_stopped():
            return
        if reinstall_proc.returncode in (0, -9):
            screen.write("[green][ok][/green] cobot reinstalled")
        else:
            screen.write(f"[yellow]Warning:[/yellow] reinstall failed — {reinstall_err.strip()}")

        if not screen.is_stopped():
            screen.set_progress(100, "Done")
            screen.write("\n[green]Project updated successfully.[/green]")
            screen.finish(True)

    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


class _UpdateApp(App[None]):
    """Minimal Textual app that opens a LogScreen running _task_update and exits when it closes.
    Минимальное Textual-приложение, открывающее LogScreen с _task_update и завершающееся при закрытии.
    """

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
