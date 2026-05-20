from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from rich.console import Console
from rich.progress import BarColumn, Progress, SpinnerColumn, TaskProgressColumn, TextColumn
from textual.app import App

from cobot.tui import SCREEN_CSS, InputScreen, PickScreen

_console = Console()

_PROJECT_DIR = Path(__file__).parent.parent.parent
_DOCKER_DIR = _PROJECT_DIR / "docker"
_DEFAULT_HUB_REPO = "evilfisru/lwc"
_DEFAULT_PREFIX = "lwc-local"

_CONTROLLER_CHAIN = ["ros-core", "ros-base", "ros-iiwa7"]
_WEBOTS_CHAIN = ["ros-core", "ros-base", "ros-iiwa7-webots"]

_IMAGE_PARENT: dict[str, str | None] = {
    "ros-core": None,
    "ros-base": "ros-core",
    "ros-iiwa7": "ros-base",
    "ros-iiwa7-webots": "ros-base",
}

# Images that COPY from src/ — need project root as build context.
# Others use their Dockerfile's own directory.
_NEEDS_PROJECT_CTX = {"ros-iiwa7", "ros-iiwa7-webots"}


@dataclass
class _Config:
    ros_version: str
    variant: str      # "controller" | "webots"
    source: str       # "build" | "pull"
    build_type: str   # "release" | "dev"
    image_prefix: str
    hub_repo: str


class _Wizard(App[Optional[_Config]]):
    CSS = SCREEN_CSS

    def __init__(self, versions: List[str], default_version: str = "jazzy"):
        super().__init__()
        self.versions = versions
        self.default_version = default_version
        self._state: dict = {}

    def on_mount(self) -> None:
        self._ask_version()

    # ── step helpers ──────────────────────────────────────────────────────────

    def _ask_version(self) -> None:
        self.push_screen(
            PickScreen("Step 1 of 5", "Select ROS version:", self.versions, self.default_version),
            self._got_version,
        )

    def _got_version(self, v: Optional[str]) -> None:
        if v is None:
            self.exit(None)
            return
        self._state["ros_version"] = v
        self.push_screen(
            PickScreen("Step 2 of 5", "Source:", ["Pull from Docker Hub", "Build locally"], "Pull from Docker Hub"),
            self._got_source,
        )

    def _got_source(self, v: Optional[str]) -> None:
        if v is None:
            self.exit(None)
            return
        self._state["source"] = "build" if v == "Build locally" else "pull"
        self.push_screen(
            PickScreen("Step 3 of 5", "What to install:", ["Controller only", "Controller and Webots"], "Controller only"),
            self._got_variant,
        )

    def _got_variant(self, v: Optional[str]) -> None:
        if v is None:
            self.exit(None)
            return
        self._state["variant"] = "webots" if v == "Controller and Webots" else "controller"
        self.push_screen(
            PickScreen("Step 4 of 5", "Build type:", ["release", "dev"], "release"),
            self._got_build_type,
        )

    def _got_build_type(self, v: Optional[str]) -> None:
        if v is None:
            self.exit(None)
            return
        self._state["build_type"] = v or "release"
        if self._state["source"] == "pull":
            self.push_screen(
                InputScreen("Step 5 of 5", "Docker Hub repository:", _DEFAULT_HUB_REPO),
                self._got_hub_repo,
            )
        else:
            self.push_screen(
                InputScreen("Step 5 of 5", "Image prefix:", _DEFAULT_PREFIX),
                self._got_prefix,
            )

    def _got_hub_repo(self, v: Optional[str]) -> None:
        if v is None:
            self.exit(None)
            return
        self._state["hub_repo"] = v
        self._finish()

    def _got_prefix(self, v: Optional[str]) -> None:
        if v is None:
            self.exit(None)
            return
        self._state["image_prefix"] = v
        self._finish()

    def _finish(self) -> None:
        s = self._state
        self.exit(_Config(
            ros_version=s["ros_version"],
            variant=s["variant"],
            source=s["source"],
            build_type=s["build_type"],
            image_prefix=s.get("image_prefix", _DEFAULT_PREFIX),
            hub_repo=s.get("hub_repo", _DEFAULT_HUB_REPO),
        ))


def _discover_versions() -> List[str]:
    if not _DOCKER_DIR.exists():
        return ["jazzy"]
    dirs = sorted(d.name for d in _DOCKER_DIR.iterdir() if d.is_dir())
    if "jazzy" in dirs:
        dirs = ["jazzy"] + [d for d in dirs if d != "jazzy"]
    return dirs or ["jazzy"]


def _build_image(
    name: str, tag: str, dockerfile: Path, ctx: Path,
    parent_tag: Optional[str] = None,
    build_type: str = "release",
) -> bool:
    env = {**os.environ, "DOCKER_BUILDKIT": "0"}
    cmd = [
        "docker", "build",
        "-t", tag,
        "-f", str(dockerfile),
        "--build-arg", f"BUILD_TYPE={build_type}",
    ]
    if parent_tag:
        cmd += ["--build-arg", f"IMAGE={parent_tag}"]
    cmd.append(str(ctx))
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=env,
    )

    captured: List[str] = []

    with Progress(
        SpinnerColumn(),
        TextColumn(f"  [bold cyan]{name:<24}[/bold cyan]"),
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
        _console.print(f"\n[red]Build failed:[/red] {tag}\n")
        _console.print("".join(captured), highlight=False)
        return False
    return True


def _parse_docker_size(s: str) -> float:
    s = s.strip()
    for suffix, mult in [("GB", 1e9), ("MB", 1e6), ("kB", 1e3), ("B", 1.0)]:
        if s.endswith(suffix):
            try:
                return float(s[: -len(suffix)]) * mult
            except ValueError:
                return 0.0
    try:
        return float(s)
    except ValueError:
        return 0.0


_RE_PULLING = re.compile(r"^([a-f0-9]+): Pulling fs layer")
_RE_DONE = re.compile(r"^([a-f0-9]+): (?:Pull complete|Already exists|Layer already exists)")
_RE_DL = re.compile(r"^([a-f0-9]+): Downloading(?:\s+\[.*?\])?\s+([\d.]+\s*\w+)/([\d.]+\s*\w+)")


def _pull_image(name: str, tag: str) -> bool:
    proc = subprocess.Popen(
        ["docker", "pull", tag],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True,
    )

    captured: List[str] = []
    layers_pulling: set[str] = set()
    layers_done: set[str] = set()
    layers_total: dict[str, float] = {}
    layers_current: dict[str, float] = {}
    has_bytes = False

    with Progress(
        SpinnerColumn(),
        TextColumn(f"  [bold cyan]{name:<24}[/bold cyan]"),
        BarColumn(bar_width=32),
        TaskProgressColumn(),
        console=_console,
        transient=False,
    ) as prog:
        task = prog.add_task("", total=100)

        for line in proc.stdout:
            captured.append(line)
            line = line.strip()

            if m := _RE_PULLING.match(line):
                layers_pulling.add(m.group(1))

            elif m := _RE_DL.match(line):
                lid, cur, tot = m.group(1), _parse_docker_size(m.group(2)), _parse_docker_size(m.group(3))
                if tot > 0:
                    has_bytes = True
                    layers_current[lid] = cur
                    layers_total[lid] = tot

            elif m := _RE_DONE.match(line):
                lid = m.group(1)
                layers_done.add(lid)
                if lid in layers_total:
                    layers_current[lid] = layers_total[lid]

            if has_bytes and layers_total:
                total_b = sum(layers_total.values())
                done_b = sum(layers_current.get(lid, 0.0) for lid in layers_total)
                prog.update(task, completed=done_b / total_b * 100)
            elif layers_pulling:
                prog.update(task, completed=len(layers_done) / len(layers_pulling) * 100)

        prog.update(task, completed=100)

    proc.wait()
    if proc.returncode != 0:
        _console.print(f"\n[red]Pull failed:[/red] {tag}\n")
        _console.print("".join(captured), highlight=False)
        return False
    return True


def _execute(cfg: _Config) -> None:
    chain = _WEBOTS_CHAIN if cfg.variant == "webots" else _CONTROLLER_CHAIN
    _console.print()

    if cfg.source == "build":
        _console.print(
            f"[bold]Building {len(chain)} image(s)  •  ROS {cfg.ros_version}  •  {cfg.build_type}[/bold]\n"
        )
        for name in chain:
            tag = f"{cfg.image_prefix}:{name}-{cfg.ros_version}"
            dockerfile = _DOCKER_DIR / cfg.ros_version / name / "Dockerfile"
            if not dockerfile.exists():
                _console.print(f"[red]Dockerfile not found:[/red] {dockerfile}")
                sys.exit(1)
            ctx = _PROJECT_DIR if name in _NEEDS_PROJECT_CTX else dockerfile.parent
            parent_name = _IMAGE_PARENT.get(name)
            parent_tag = f"{cfg.image_prefix}:{parent_name}-{cfg.ros_version}" if parent_name else None
            if not _build_image(name, tag, dockerfile, ctx, parent_tag, cfg.build_type):
                sys.exit(1)
        _console.print(f"\n[green]Done.[/green] Images: [bold]{cfg.image_prefix}:{{name}}-{cfg.ros_version}[/bold].")

    else:
        short = "webots" if cfg.variant == "webots" else "iiwa"
        suffix = "-dev" if cfg.build_type == "dev" else ""
        image_tag = f"{short}-{cfg.ros_version}{suffix}"
        full_ref = f"{cfg.hub_repo}:{image_tag}"
        _console.print(f"[bold]Pulling from {cfg.hub_repo}  •  ROS {cfg.ros_version}  •  {cfg.build_type}[/bold]\n")
        if not _pull_image(short, full_ref):
            sys.exit(1)
        _console.print(f"\n[green]Done.[/green] Image ready: [bold]{full_ref}[/bold].")


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("docker-setup", help="Set up Docker images for KUKA iiwa7")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    if not shutil.which("docker"):
        _console.print("[red]Error:[/red] Docker is not installed or not on PATH.")
        sys.exit(1)

    versions = _discover_versions()
    default = "jazzy" if "jazzy" in versions else versions[0]
    cfg = _Wizard(versions=versions, default_version=default).run()

    if cfg is None:
        return

    _execute(cfg)
