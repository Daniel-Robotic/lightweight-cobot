from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

from textual.app import App

from cobot.tui import SCREEN_CSS, LogScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent


def _task_update(screen: LogScreen) -> None:
    try:
        # Current branch
        branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=_PROJECT_DIR, text=True,
        ).strip()
        screen.write(f"[cyan][*][/cyan] Branch: [bold]{branch}[/bold]")

        # Fetch
        screen.write("[cyan][*][/cyan] Fetching from remote...")
        fetch = subprocess.run(
            ["git", "fetch", "origin"],
            cwd=_PROJECT_DIR, capture_output=True, text=True,
        )
        if fetch.returncode != 0:
            screen.write(f"[red]Fetch failed:[/red] {fetch.stderr.strip()}")
            screen.finish(False)
            return

        # Check how many commits behind
        behind = subprocess.check_output(
            ["git", "rev-list", f"HEAD..origin/{branch}", "--count"],
            cwd=_PROJECT_DIR, text=True,
        ).strip()

        if behind == "0":
            screen.write("[green][ok][/green] Already up to date.")
            screen.finish(True)
            return

        # Show incoming commits
        screen.write(f"\n[bold]{behind} new commit(s):[/bold]")
        log_lines = subprocess.check_output(
            ["git", "log", f"HEAD..origin/{branch}", "--oneline"],
            cwd=_PROJECT_DIR, text=True,
        ).strip().splitlines()
        for line in log_lines:
            screen.write(f"  [dim]{line}[/dim]")

        # Pull
        screen.write("\n[cyan][*][/cyan] Pulling changes...")
        proc = subprocess.Popen(
            ["git", "pull", "origin", branch],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, cwd=_PROJECT_DIR,
        )
        for line in proc.stdout:
            s = line.rstrip()
            if s:
                screen.write(s)
        proc.wait()
        if proc.returncode != 0:
            screen.write("[red]Pull failed.[/red]")
            screen.finish(False)
            return

        # Reinstall in case dependencies changed
        screen.write("\n[cyan][*][/cyan] Reinstalling cobot CLI...")
        reinstall = subprocess.run(
            ["uv", "tool", "install", "--editable", str(_PROJECT_DIR)],
            capture_output=True, text=True,
        )
        if reinstall.returncode == 0:
            screen.write("[green][ok][/green] cobot reinstalled")
        else:
            screen.write(f"[yellow]Warning:[/yellow] reinstall failed — {reinstall.stderr.strip()}")

        screen.write("\n[green]Project updated successfully.[/green]")
        screen.finish(True)

    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)


class _UpdateApp(App[None]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            LogScreen("Updating project", _task_update),
            lambda _: self.exit(),
        )


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("update", help="Pull latest changes from the remote git branch")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    _UpdateApp().run()
