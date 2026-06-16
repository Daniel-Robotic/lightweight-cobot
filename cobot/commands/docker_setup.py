from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from cobot import process, ui
from cobot.ui import done
from cobot.process import StepProgress

_PROJECT_DIR = Path(__file__).parent.parent.parent
_DOCKER_DIR = _PROJECT_DIR / "docker"

# Default Docker Hub repository and local image prefix used when building locally.
# Репозиторий Docker Hub по умолчанию и локальный префикс образов при локальной сборке.
_DEFAULT_HUB_REPO = "evilfisru/lwc"
_DEFAULT_PREFIX = "lwc-local"

# The images must be built in this order because each one is based on the previous.
# Образы должны собираться в этом порядке, потому что каждый основан на предыдущем.
_CONTROLLER_CHAIN = ["ros-core", "ros-base", "ros-iiwa7"]
_WEBOTS_CHAIN = ["ros-core", "ros-base", "ros-iiwa7-webots"]

# Maps each image to the image it is built FROM. None means it starts from base Ubuntu.
# Сопоставляет каждый образ с тем, на основе которого он собирается. None - базовый Ubuntu.
_IMAGE_PARENT: dict[str, str | None] = {
    "ros-core": None,
    "ros-base": "ros-core",
    "ros-iiwa7": "ros-base",
    "ros-iiwa7-webots": "ros-base",
}

# These images need the full project source as Docker build context.
# Эти образы требуют полный исходный код проекта как контекст сборки.
_NEEDS_PROJECT_CTX = {"ros-iiwa7", "ros-iiwa7-webots"}


@dataclass
class _Config:
    ros_version: str
    variant: str
    source: str
    build_type: str
    image_prefix: str
    hub_repo: str


def _build_image(name: str, tag: str, dockerfile: Path, ctx: Path, p: StepProgress,
                 lo: float, hi: float, parent_tag: Optional[str], build_type: str) -> bool:
    """Build a single Docker image, streaming output and mapping "Step X/Y" to the
    lo..hi slice of the progress bar. Returns True on success.
    Собирает один Docker-образ, транслируя вывод и отображая "Step X/Y" на участок
    lo..hi прогресс-бара. Возвращает True при успехе.
    """
    p.raw(f"[cyan]▸[/cyan] Сборка [bold]{name}[/bold]...")
    env = {**os.environ, "DOCKER_BUILDKIT": "0"}
    cmd = [
        "docker", "build", "-t", tag, "-f", str(dockerfile),
        "--build-arg", f"BUILD_TYPE={build_type}",
    ]
    if parent_tag:
        cmd += ["--build-arg", f"IMAGE={parent_tag}"]
    cmd.append(str(ctx))

    def on_line(s: str) -> None:
        if s:
            p.log(s)
        m = re.match(r"Step (\d+)/(\d+) :", s)
        if m:
            step, total = int(m.group(1)), int(m.group(2))
            p.set(lo + step / total * (hi - lo), f"{name}: шаг {step}/{total}")

    rc = process.stream(cmd, env=env, on_line=on_line)
    if rc in (-9, -15):
        return False
    if rc == 0:
        p.raw(f"[green]✓[/green] {name}")
        return True
    p.raw(f"[red]Сборка не удалась:[/red] {tag}")
    return False


def _pull_image(name: str, tag: str, p: StepProgress, lo: float, hi: float) -> bool:
    """Pull a Docker image, tracking progress by counting completed layers.
    Скачивает Docker-образ, отслеживая прогресс по числу завершённых слоёв.
    """
    p.raw(f"[cyan]▸[/cyan] Скачивание [bold]{name}[/bold]  ({tag})...")
    layers_total = 0
    layers_done = 0

    def on_line(s: str) -> None:
        nonlocal layers_total, layers_done
        if s:
            p.log(s)
        if "Pulling fs layer" in s or "Waiting" in s:
            layers_total += 1
        elif "Pull complete" in s or "Already exists" in s:
            layers_done += 1
            if layers_total > 0:
                p.set(lo + layers_done / layers_total * (hi - lo), f"{name}: слои")

    rc = process.stream(["docker", "pull", tag], on_line=on_line)
    if rc in (-9, -15):
        return False
    if rc == 0:
        p.raw(f"[green]✓[/green] {name}")
        return True
    p.raw(f"[red]Скачивание не удалось:[/red] {tag}")
    return False


def _execute(cfg: _Config) -> None:
    """Build or pull all images in the chain selected by the user's choices.
    Собирает или скачивает все образы из цепочки, выбранной пользователем.
    """
    chain = _WEBOTS_CHAIN if cfg.variant == "webots" else _CONTROLLER_CHAIN
    n = len(chain)
    ok = True
    fail_msg = ""

    if cfg.source == "build":
        title = f"Сборка {n} образ(ов) — ROS {cfg.ros_version} — {cfg.build_type}"
        with StepProgress(title) as p:
            for i, name in enumerate(chain):
                lo, hi = i / n * 100, (i + 1) / n * 100
                p.set(lo, f"Образ {i + 1}/{n}: {name}")
                tag = f"{cfg.image_prefix}:{name}-{cfg.ros_version}"
                dockerfile = _DOCKER_DIR / cfg.ros_version / name / "Dockerfile"
                if not dockerfile.exists():
                    ok, fail_msg = False, f"Dockerfile не найден: {dockerfile}"
                    break
                ctx = _PROJECT_DIR if name in _NEEDS_PROJECT_CTX else dockerfile.parent
                parent_name = _IMAGE_PARENT.get(name)
                parent_tag = (f"{cfg.image_prefix}:{parent_name}-{cfg.ros_version}"
                              if parent_name else None)
                if not _build_image(name, tag, dockerfile, ctx, p, lo, hi,
                                    parent_tag, cfg.build_type):
                    ok, fail_msg = False, f"Сборка образа {name} не удалась"
                    break
            if ok:
                p.set(100, "Готово")
        done(ok, f"Образы готовы: {cfg.image_prefix}:<name>-{cfg.ros_version}"
             if ok else fail_msg)
    else:
        short = "webots" if cfg.variant == "webots" else "iiwa"
        suffix = "-dev" if cfg.build_type == "dev" else ""
        full_ref = f"{cfg.hub_repo}:{short}-{cfg.ros_version}{suffix}"
        with StepProgress(f"Скачивание из {cfg.hub_repo} — ROS {cfg.ros_version}") as p:
            if _pull_image(short, full_ref, p, 0, 100):
                p.set(100, "Готово")
            else:
                ok, fail_msg = False, f"Скачивание {full_ref} не удалось"
        done(ok, f"Образ готов: {full_ref}" if ok else fail_msg)


def _discover_versions() -> List[str]:
    """Return ROS versions found in docker/ (jazzy first). Falls back to ["jazzy"].
    Возвращает версии ROS из docker/ (jazzy первым). По умолчанию ["jazzy"].
    """
    if not _DOCKER_DIR.exists():
        return ["jazzy"]
    dirs = sorted(d.name for d in _DOCKER_DIR.iterdir() if d.is_dir())
    if "jazzy" in dirs:
        dirs = ["jazzy"] + [d for d in dirs if d != "jazzy"]
    return dirs or ["jazzy"]


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("docker-setup", help="Build or pull Docker images for KUKA iiwa7")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    if not shutil.which("docker"):
        ui.error("Docker не установлен или отсутствует в PATH.")
        sys.exit(1)

    ui.header("Настройка Docker", "сборка или скачивание образов KUKA iiwa7")

    versions = _discover_versions()
    default = "jazzy" if "jazzy" in versions else versions[0]

    ros_version = ui.select("Версия ROS:", versions, default)
    if ros_version is None:
        return

    src = ui.select("Источник:", ["Скачать с Docker Hub", "Собрать локально"],
                    "Скачать с Docker Hub")
    if src is None:
        return
    source = "build" if src == "Собрать локально" else "pull"

    variant_v = ui.select(
        "Что установить:",
        ["Только контроллер — ros-core, ros-base, ros-iiwa7",
         "Контроллер с Webots — ros-core, ros-base, ros-iiwa7-webots"],
        "Только контроллер — ros-core, ros-base, ros-iiwa7",
    )
    if variant_v is None:
        return
    variant = "webots" if variant_v.startswith("Контроллер с Webots") else "controller"

    build_type = ui.select("Тип сборки:", ["release", "dev"], "release")
    if build_type is None:
        return

    image_prefix, hub_repo = _DEFAULT_PREFIX, _DEFAULT_HUB_REPO
    if source == "pull":
        v = ui.text("Репозиторий Docker Hub:", _DEFAULT_HUB_REPO)
        if v is None:
            return
        hub_repo = v
    else:
        v = ui.text("Префикс образов:", _DEFAULT_PREFIX)
        if v is None:
            return
        image_prefix = v

    cfg = _Config(
        ros_version=ros_version, variant=variant, source=source,
        build_type=build_type, image_prefix=image_prefix, hub_repo=hub_repo,
    )
    _execute(cfg)
