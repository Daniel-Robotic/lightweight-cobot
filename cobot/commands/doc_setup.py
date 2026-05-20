from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Callable, Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, InputScreen, LogScreen

_PROJECT_DIR = Path(__file__).parent.parent.parent
_DOC_DIR = _PROJECT_DIR / "doc" / "lwc-doc"
_IMAGE_NAME = "lwc-docs"
_CONTAINER_NAME = "lwc-docs"
_DEFAULT_PORT = "8000"

Write = Callable[[str], None]



def _docker(*args: str, capture: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(["docker", *args], capture_output=capture, text=True)


def _is_running() -> bool:
    r = _docker("ps", "--filter", f"name={_CONTAINER_NAME}", "--format", "{{.Names}}", capture=True)
    return _CONTAINER_NAME in r.stdout


def _image_exists() -> bool:
    return bool(_docker("images", "-q", _IMAGE_NAME, capture=True).stdout.strip())


def _build_docs_image(write: Write) -> bool:
    write("[cyan][*][/cyan] Building documentation image (runs once)...")
    env = {**os.environ, "DOCKER_BUILDKIT": "0"}
    result = subprocess.run(
        ["docker", "build", "-t", _IMAGE_NAME, str(_DOC_DIR)],
        capture_output=True, text=True, env=env,
    )
    if result.returncode != 0:
        for line in (result.stdout + result.stderr).splitlines():
            if line.strip():
                write(line)
        write("[red]Image build failed.[/red]")
        return False
    write("[green][ok][/green] Documentation image ready")
    return True



def _task_up(screen: LogScreen, port: str) -> None:
    try:
        if _is_running():
            screen.write(f"[green]Docs already running at:[/green] http://localhost:{port}")
            screen.write("  Stop with: [bold]cobot doc-setup down[/bold]")
            screen.finish(True)
            return

        if not _DOC_DIR.exists():
            screen.write(f"[red]Doc directory not found:[/red] {_DOC_DIR}")
            screen.finish(False)
            return

        if not _image_exists():
            if not _build_docs_image(screen.write):
                screen.finish(False)
                return
        else:
            screen.write("[dim]Documentation image already built, skipping.[/dim]")

        screen.write("\n[cyan][*][/cyan] Starting MkDocs server...")
        result = _docker(
            "run", "-d", "--name", _CONTAINER_NAME, "--rm",
            "-p", f"{port}:8000",
            "-v", f"{_DOC_DIR}:/docs",
            _IMAGE_NAME, "serve", "--dev-addr=0.0.0.0:8000",
            capture=True,
        )
        if result.returncode != 0:
            screen.write(f"[red]Failed to start container.[/red]\n{result.stderr}")
            screen.finish(False)
            return

        screen.write(f"\n[green]Docs running at:[/green] http://localhost:{port}")
        screen.write("  Edit files in [bold]doc/lwc-doc/docs/[/bold] — reloads automatically.")
        screen.write("  Stop with: [bold]cobot doc-setup down[/bold]")
        screen.finish(True)

    except Exception as exc:
        screen.write(f"[red]Error:[/red] {exc}")
        screen.finish(False)


def _task_down(screen: LogScreen) -> None:
    try:
        if not _is_running():
            screen.write("[yellow]Docs container is not running.[/yellow]")
            screen.finish(True)
            return
        screen.write("[cyan][*][/cyan] Stopping documentation server...")
        _docker("stop", _CONTAINER_NAME)
        screen.write("[green][ok][/green] Container stopped.")
        screen.finish(True)
    except Exception as exc:
        screen.write(f"[red]Error:[/red] {exc}")
        screen.finish(False)


def _task_rebuild(screen: LogScreen, port: str) -> None:
    try:
        if _is_running():
            screen.write("[cyan][*][/cyan] Stopping existing container...")
            _docker("stop", _CONTAINER_NAME)
            screen.write("[green][ok][/green] Stopped.")

        if _image_exists():
            screen.write("[cyan][*][/cyan] Removing old image...")
            _docker("rmi", "-f", _IMAGE_NAME)
            screen.write("[green][ok][/green] Image removed.")

        if not _build_docs_image(screen.write):
            screen.finish(False)
            return

        screen.write("\n[cyan][*][/cyan] Starting MkDocs server...")
        result = _docker(
            "run", "-d", "--name", _CONTAINER_NAME, "--rm",
            "-p", f"{port}:8000",
            "-v", f"{_DOC_DIR}:/docs",
            _IMAGE_NAME, "serve", "--dev-addr=0.0.0.0:8000",
            capture=True,
        )
        if result.returncode != 0:
            screen.write(f"[red]Failed to start container.[/red]\n{result.stderr}")
            screen.finish(False)
            return

        screen.write(f"\n[green]Docs running at:[/green] http://localhost:{port}")
        screen.write("  Stop with: [bold]cobot doc-setup down[/bold]")
        screen.finish(True)

    except Exception as exc:
        screen.write(f"[red]Error:[/red] {exc}")
        screen.finish(False)


class _DocApp(App[None]):
    CSS = SCREEN_CSS

    def __init__(self, action: str):
        super().__init__()
        self._action = action

    def on_mount(self) -> None:
        if self._action == "down":
            self.push_screen(
                LogScreen("Documentation server", _task_down),
                lambda _: self.exit(),
            )
        elif self._action == "rebuild":
            self.push_screen(
                InputScreen("Step 1 of 1", "Port to serve documentation on:", _DEFAULT_PORT),
                self._got_port_rebuild,
            )
        else:
            self.push_screen(
                InputScreen("Step 1 of 1", "Port to serve documentation on:", _DEFAULT_PORT),
                self._got_port_up,
            )

    def _got_port_up(self, port: Optional[str]) -> None:
        if port is None:
            self.exit()
            return
        p = (port.strip() or _DEFAULT_PORT) if port.isdigit() or not port.strip() else _DEFAULT_PORT
        self.push_screen(
            LogScreen("Documentation server", lambda s: _task_up(s, p)),
            lambda _: self.exit(),
        )

    def _got_port_rebuild(self, port: Optional[str]) -> None:
        if port is None:
            self.exit()
            return
        p = (port.strip() or _DEFAULT_PORT) if port.isdigit() or not port.strip() else _DEFAULT_PORT
        self.push_screen(
            LogScreen("Documentation server — rebuild", lambda s: _task_rebuild(s, p)),
            lambda _: self.exit(),
        )


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
        from rich.console import Console
        Console().print("[red]Error:[/red] Docker is not installed or not on PATH.")
        sys.exit(1)

    action = getattr(args, "action", "up")
    _DocApp(action).run()
