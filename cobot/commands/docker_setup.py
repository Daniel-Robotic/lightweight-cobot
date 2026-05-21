from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, InputScreen, LogScreen, PickScreen

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

_NEEDS_PROJECT_CTX = {"ros-iiwa7", "ros-iiwa7-webots"}

Write = Callable[[str], None]


@dataclass
class _Config:
    ros_version: str
    variant: str
    source: str
    build_type: str
    image_prefix: str
    hub_repo: str


def _build_image(
    name: str,
    tag: str,
    dockerfile: Path,
    ctx: Path,
    write: Write,
    on_progress: Optional[Callable[[float], None]] = None,
    parent_tag: Optional[str] = None,
    build_type: str = "release",
) -> bool:
    write(f"[cyan][*][/cyan] Building [bold]{name}[/bold]...")
    env = {**os.environ, "DOCKER_BUILDKIT": "0"}
    cmd = [
        "docker", "build", "-t", tag, "-f", str(dockerfile),
        "--build-arg", f"BUILD_TYPE={build_type}",
    ]
    if parent_tag:
        cmd += ["--build-arg", f"IMAGE={parent_tag}"]
    cmd.append(str(ctx))

    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
    )
    for line in proc.stdout:
        s = line.rstrip()
        if s:
            write(s)
        if on_progress:
            m = re.match(r"Step (\d+)/(\d+) :", line)
            if m:
                step, total = int(m.group(1)), int(m.group(2))
                on_progress(step / total * 100)
    proc.wait()
    if proc.returncode == 0:
        write(f"[green][ok][/green] {name}")
        return True
    write(f"[red]Build failed:[/red] {tag}")
    return False


def _pull_image(
    name: str,
    tag: str,
    write: Write,
    on_progress: Optional[Callable[[float], None]] = None,
) -> bool:
    write(f"[cyan][*][/cyan] Pulling [bold]{name}[/bold]  ({tag})...")
    if on_progress:
        on_progress(5)

    proc = subprocess.Popen(
        ["docker", "pull", tag],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    layers_total = 0
    layers_done = 0
    for line in proc.stdout:
        s = line.rstrip()
        if s:
            write(s)
        if "Pulling fs layer" in line or "Waiting" in line:
            layers_total += 1
        elif "Pull complete" in line or "Already exists" in line:
            layers_done += 1
            if on_progress and layers_total > 0:
                on_progress(5 + layers_done / layers_total * 90)
    proc.wait()
    if proc.returncode == 0:
        write(f"[green][ok][/green] {name}")
        if on_progress:
            on_progress(100)
        return True
    write(f"[red]Pull failed:[/red] {tag}")
    return False


def _task_execute(screen: LogScreen, cfg: _Config) -> None:
    try:
        chain = _WEBOTS_CHAIN if cfg.variant == "webots" else _CONTROLLER_CHAIN
        n = len(chain)

        if cfg.source == "build":
            screen.write(
                f"[bold]Building {n} image(s) — "
                f"ROS {cfg.ros_version} — {cfg.build_type}[/bold]\n"
            )
            for i, name in enumerate(chain):
                lo = i / n * 100
                hi = (i + 1) / n * 100
                screen.set_progress(lo, f"Image {i + 1}/{n}: building {name}...")

                tag = f"{cfg.image_prefix}:{name}-{cfg.ros_version}"
                dockerfile = _DOCKER_DIR / cfg.ros_version / name / "Dockerfile"
                if not dockerfile.exists():
                    screen.write(f"[red]Dockerfile not found:[/red] {dockerfile}")
                    screen.finish(False)
                    return
                ctx = _PROJECT_DIR if name in _NEEDS_PROJECT_CTX else dockerfile.parent
                parent_name = _IMAGE_PARENT.get(name)
                parent_tag = (
                    f"{cfg.image_prefix}:{parent_name}-{cfg.ros_version}"
                    if parent_name else None
                )
                if not _build_image(
                    name, tag, dockerfile, ctx, screen.write,
                    on_progress=lambda p, lo=lo, hi=hi: screen.set_progress(
                        lo + p * (hi - lo) / 100, f"Image {i + 1}/{n}: building {name}..."
                    ),
                    parent_tag=parent_tag,
                    build_type=cfg.build_type,
                ):
                    screen.finish(False)
                    return
                screen.set_progress(hi)

            screen.set_progress(100, "All images built")
            screen.write(
                f"\n[green]Done.[/green] "
                f"Images tagged [bold]{cfg.image_prefix}:<name>-{cfg.ros_version}[/bold]."
            )

        else:
            short = "webots" if cfg.variant == "webots" else "iiwa"
            suffix = "-dev" if cfg.build_type == "dev" else ""
            full_ref = f"{cfg.hub_repo}:{short}-{cfg.ros_version}{suffix}"
            screen.write(
                f"[bold]Pulling from {cfg.hub_repo} — "
                f"ROS {cfg.ros_version} — {cfg.build_type}[/bold]\n"
            )
            screen.set_progress(0, f"Pulling {full_ref}...")
            if not _pull_image(
                short, full_ref, screen.write,
                on_progress=lambda p: screen.set_progress(p, f"Pulling {full_ref}..."),
            ):
                screen.finish(False)
                return
            screen.set_progress(100, "Pull complete")
            screen.write(f"\n[green]Done.[/green] Image ready: [bold]{full_ref}[/bold].")

        screen.finish(True)

    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)


def _discover_versions() -> List[str]:
    if not _DOCKER_DIR.exists():
        return ["jazzy"]
    dirs = sorted(d.name for d in _DOCKER_DIR.iterdir() if d.is_dir())
    if "jazzy" in dirs:
        dirs = ["jazzy"] + [d for d in dirs if d != "jazzy"]
    return dirs or ["jazzy"]


class _Wizard(App[None]):
    CSS = SCREEN_CSS

    def __init__(self, versions: List[str], default_version: str = "jazzy"):
        super().__init__()
        self.versions = versions
        self.default_version = default_version
        self._state: dict = {}

    def on_mount(self) -> None:
        self._ask_version()

    def _ask_version(self) -> None:
        self.push_screen(
            PickScreen("Step 1 of 5", "Select ROS version:", self.versions, self.default_version),
            self._got_version,
        )

    def _got_version(self, v: Optional[str]) -> None:
        if v is None:
            self.exit()
            return
        self._state["ros_version"] = v
        self.push_screen(
            PickScreen(
                "Step 2 of 5", "Source:",
                ["Pull from Docker Hub", "Build locally"],
                "Pull from Docker Hub",
            ),
            self._got_source,
        )

    def _got_source(self, v: Optional[str]) -> None:
        if v is None:
            self.exit()
            return
        self._state["source"] = "build" if v == "Build locally" else "pull"
        self.push_screen(
            PickScreen(
                "Step 3 of 5",
                "What to install:",
                [
                    "Controller only — ros-core, ros-base, ros-iiwa7",
                    "Controller with Webots — ros-core, ros-base, ros-iiwa7-webots",
                ],
                "Controller only — ros-core, ros-base, ros-iiwa7",
            ),
            self._got_variant,
        )

    def _got_variant(self, v: Optional[str]) -> None:
        if v is None:
            self.exit()
            return
        self._state["variant"] = "webots" if v.startswith("Controller with Webots") else "controller"
        self.push_screen(
            PickScreen("Step 4 of 5", "Build type:", ["release", "dev"], "release"),
            self._got_build_type,
        )

    def _got_build_type(self, v: Optional[str]) -> None:
        if v is None:
            self.exit()
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
            self.exit()
            return
        self._state["hub_repo"] = v
        self._finish()

    def _got_prefix(self, v: Optional[str]) -> None:
        if v is None:
            self.exit()
            return
        self._state["image_prefix"] = v
        self._finish()

    def _finish(self) -> None:
        s = self._state
        cfg = _Config(
            ros_version=s["ros_version"],
            variant=s["variant"],
            source=s["source"],
            build_type=s["build_type"],
            image_prefix=s.get("image_prefix", _DEFAULT_PREFIX),
            hub_repo=s.get("hub_repo", _DEFAULT_HUB_REPO),
        )
        title = (
            f"Building Docker images — ROS {cfg.ros_version}"
            if cfg.source == "build"
            else f"Pulling Docker image — ROS {cfg.ros_version}"
        )
        self.push_screen(
            LogScreen(title, lambda screen: _task_execute(screen, cfg), show_progress=True),
            lambda _: self.exit(),
        )


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("docker-setup", help="Build or pull Docker images for KUKA iiwa7")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    if not shutil.which("docker"):
        from rich.console import Console
        Console().print("[red]Error:[/red] Docker is not installed or not on PATH.")
        sys.exit(1)

    versions = _discover_versions()
    default = "jazzy" if "jazzy" in versions else versions[0]
    _Wizard(versions=versions, default_version=default).run()
