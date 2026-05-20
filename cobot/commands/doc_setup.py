from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import List, Optional

from rich.console import Console
from rich.progress import BarColumn, Progress, SpinnerColumn, TaskProgressColumn, TextColumn
from textual.app import App

from cobot.tui import SCREEN_CSS, InputScreen

_console = Console()

_PROJECT_DIR = Path(__file__).parent.parent.parent
_DOC_DIR = _PROJECT_DIR / "doc" / "lwc-doc"
_IMAGE_NAME = "lwc-docs"
_CONTAINER_NAME = "lwc-docs"
_DEFAULT_PORT = "8000"



class _Wizard(App[Optional[str]]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            InputScreen("Step 1 of 1", "Port to serve documentation on:", _DEFAULT_PORT),
            self._got_port,
        )

    def _got_port(self, v: Optional[str]) -> None:
        if v is None:
            self.exit(None)
            return
        port = v.strip() or _DEFAULT_PORT
        if not port.isdigit():
            self.exit(_DEFAULT_PORT)
        else:
            self.exit(port)



def _docker(*args: str, capture: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["docker", *args],
        capture_output=capture,
        text=True,
    )


def _is_running() -> bool:
    result = _docker(
        "ps", "--filter", f"name={_CONTAINER_NAME}", "--format", "{{.Names}}",
        capture=True,
    )
    return _CONTAINER_NAME in result.stdout


def _image_exists() -> bool:
    result = _docker("images", "-q", _IMAGE_NAME, capture=True)
    return bool(result.stdout.strip())


def _build_docs_image() -> bool:
    _console.print("[dim]Installing MkDocs plugins into the image (this runs once)...[/dim]")
    env = {**os.environ, "DOCKER_BUILDKIT": "0"}
    proc = subprocess.Popen(
        ["docker", "build", "-t", _IMAGE_NAME, str(_DOC_DIR)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=env,
    )

    captured: List[str] = []

    with Progress(
        SpinnerColumn(),
        TextColumn("  [bold cyan]lwc-docs[/bold cyan]"),
        BarColumn(bar_width=32),
        TaskProgressColumn(),
        console=_console,
        transient=False,
    ) as prog:
        task = prog.add_task("", total=100)
        total = 1
        for line in proc.stdout:
            captured.append(line)
            m = re.match(r"Step (\d+)/(\d+) :", line)
            if m:
                step, total = int(m.group(1)), int(m.group(2))
                prog.update(task, completed=step / total * 100)
        prog.update(task, completed=100)

    proc.wait()
    if proc.returncode != 0:
        _console.print("\n[red]Image build failed:[/red]")
        _console.print("".join(captured), highlight=False)
        return False
    return True



def _cmd_up() -> None:
    if _is_running():
        _console.print(f"[green]Docs are already running at:[/green] http://localhost:{_DEFAULT_PORT}")
        _console.print("  Stop with: [bold]cobot doc-setup down[/bold]")
        return

    if not _DOC_DIR.exists():
        _console.print(f"[red]Doc directory not found:[/red] {_DOC_DIR}")
        sys.exit(1)

    port = _Wizard().run()
    if port is None:
        return

    _console.print()

    if not _image_exists():
        _console.print("[bold]Building documentation image...[/bold]")
        if not _build_docs_image():
            sys.exit(1)
        _console.print()
    else:
        _console.print("[dim]Documentation image already built, skipping...[/dim]\n")

    _console.print("[bold]Starting MkDocs server...[/bold]")
    _console.print(f"[dim]Mounting {_DOC_DIR} into container on port {port}...[/dim]")

    result = _docker(
        "run", "-d",
        "--name", _CONTAINER_NAME,
        "--rm",
        "-p", f"{port}:8000",
        "-v", f"{_DOC_DIR}:/docs",
        _IMAGE_NAME,
        "serve", "--dev-addr=0.0.0.0:8000",
    )
    if result.returncode != 0:
        _console.print("[red]Failed to start container.[/red]")
        sys.exit(1)

    _console.print(f"\n[green]Docs running at:[/green] http://localhost:{port}")
    _console.print("  Edit files in [bold]doc/lwc-doc/docs/[/bold] — the site reloads automatically.")
    _console.print("  Stop with: [bold]cobot doc-setup down[/bold]")


def _cmd_down() -> None:
    if not _is_running():
        _console.print("[yellow]Docs container is not running.[/yellow]")
        return

    _console.print("[bold]Stopping documentation server...[/bold]")
    _docker("stop", _CONTAINER_NAME)
    _console.print("[green]Done.[/green] Container removed.")



def _cmd_rebuild() -> None:
    _cmd_down()
    result = _docker("rmi", "-f", _IMAGE_NAME, capture=True)
    if result.returncode != 0:
        _console.print("[yellow]No image to remove, building fresh.[/yellow]")
    _cmd_up()


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
        _console.print("[red]Error:[/red] Docker is not installed or not on PATH.")
        sys.exit(1)

    action = getattr(args, "action", "up")
    if action == "down":
        _cmd_down()
    elif action == "rebuild":
        _cmd_rebuild()
    else:
        _cmd_up()
