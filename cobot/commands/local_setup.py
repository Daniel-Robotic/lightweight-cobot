from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path

from cobot import process, ui
from cobot import privilege
from cobot.ui import done, header
from cobot.commands.docker_setup import run as _docker_setup

# Root directory of the project, used as the working directory for colcon builds.
# Корневая директория проекта, используется как рабочая директория для сборки colcon.
_PROJECT_DIR = Path(__file__).parent.parent.parent

# ROS2 distribution name targeted by this installer.
# Название дистрибутива ROS2, который устанавливает этот скрипт.
_DISTRO = "jazzy"

# Webots simulator version targeted by this installer.
# Версия симулятора Webots, устанавливаемая этим скриптом.
_WEBOTS_VERSION = "2025a"

# Directory that contains the shell scripts used by this command.
# Директория с shell-скриптами, используемыми этой командой.
_SCRIPTS_DIR = _PROJECT_DIR / "scripts"

# apt packages that must exist before rosdep can install the pip-based keys
# (python3-pip / dev / venv). Their absence is what produced the "pip is not
# installed" failure in the screenshots.
# apt-пакеты, необходимые до того как rosdep сможет установить pip-зависимости.
# Именно их отсутствие давало ошибку "pip is not installed" на скриншотах.
_APT_PREREQS = ["python3-pip", "python3-dev", "python3-venv"]


# OS and tool detection helpers
# Вспомогательные функции для определения ОС и наличия инструментов
def _detect_ubuntu_2404() -> bool:
    """Return True if the current OS is Ubuntu 24.04 (Noble).

    Возвращает True, если текущая ОС - Ubuntu 24.04 (Noble).
    """
    path = Path("/etc/os-release")
    if not path.exists():
        return False
    info: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            info[k.strip()] = v.strip().strip('"')
    return info.get("ID") == "ubuntu" and info.get("VERSION_ID") == "24.04"


def _detect_ros2() -> bool:
    """Return True if ROS2 Jazzy is already installed under /opt/ros/jazzy.
    Возвращает True, если ROS2 Jazzy уже установлен в /opt/ros/jazzy.
    """
    return Path(f"/opt/ros/{_DISTRO}").is_dir()


def webots_installed() -> bool:
    """Return True if the Webots binary is available on PATH.
    Возвращает True, если бинарный файл Webots доступен в PATH.
    """
    return shutil.which("webots") is not None


def _ros2_env() -> dict:
    """Build an environment dict with ROS2 variables sourced from setup.bash.

    Sources /opt/ros/jazzy/setup.bash in a subprocess, captures all exported
    variables and merges them into a copy of os.environ. Falls back to plain
    os.environ if the setup file does not exist yet.

    Формирует словарь окружения с переменными ROS2, полученными из setup.bash.
    Возвращает чистый os.environ если файл setup.bash ещё не существует.
    """
    setup = Path(f"/opt/ros/{_DISTRO}/setup.bash")
    if not setup.exists():
        return os.environ.copy()
    result = subprocess.run(
        ["bash", "-c", f"source {setup} && env"],
        capture_output=True, text=True,
    )
    env = os.environ.copy()
    for line in result.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            env[k] = v

    # CMake's find_package(Python3) ignores PATH and uses its own search logic,
    # so we must pin it explicitly to the system Python where catkin_pkg is installed.
    # CMake игнорирует PATH при поиске Python через find_package(Python3),
    # поэтому явно указываем системный Python, где установлен catkin_pkg.
    env["Python3_EXECUTABLE"] = "/usr/bin/python3"
    env["PYTHON_EXECUTABLE"] = "/usr/bin/python3"

    # Keep PATH clean so other tools (rosdep, colcon itself) use system Python.
    # Чистим PATH чтобы другие инструменты тоже использовали системный Python.
    _SYSTEM_PATHS = ["/usr/bin", "/usr/local/bin"]
    existing = env.get("PATH", "").split(":")
    env["PATH"] = ":".join(
        _SYSTEM_PATHS + [p for p in existing if p not in _SYSTEM_PATHS]
    )
    return env


# Bash-script runner that understands PROGRESS:<pct>:<label> markers
# Запуск bash-скриптов с поддержкой маркеров PROGRESS:<pct>:<метка>
def _run_script(script: Path, title: str) -> int:
    """Run a shell script, streaming its output to a live log and advancing the
    progress bar from PROGRESS:<pct>:<label> markers (which are not echoed raw).
    Returns the script exit code.
    Запускает shell-скрипт, транслируя вывод в живой лог и продвигая прогресс-бар по
    маркерам PROGRESS:<pct>:<метка> (сами маркеры не печатаются). Возвращает код возврата.
    """
    if not script.exists():
        header(title)
        ui.error(f"Скрипт не найден: {script}")
        done(False, "Скрипт отсутствует")
        return 1

    with process.StepProgress(title) as p:
        def on_line(s: str) -> None:
            if s.startswith("PROGRESS:"):
                parts = s.split(":", 2)
                try:
                    p.set(float(parts[1]), parts[2] if len(parts) > 2 else "")
                except (ValueError, IndexError):
                    pass
                return
            if s:
                p.log(s)

        rc = process.stream(["bash", str(script)], cwd=str(_PROJECT_DIR), on_line=on_line)

    ok = rc in (0, -9, -15)
    done(ok, "Готово" if ok else f"Скрипт завершился с кодом {rc}")
    return rc


def install_ros2(pkg: str) -> bool:
    """Run the ROS2 Jazzy install shell script for the chosen variant (desktop / ros-base).
    Запускает shell-скрипт установки ROS2 Jazzy для выбранного варианта (desktop / ros-base).
    """
    script = _SCRIPTS_DIR / f"setup_ros2_{pkg.replace('-', '_')}.sh"
    rc = _run_script(script, f"Установка ROS2 {_DISTRO} ({pkg})")
    return rc in (0, -9, -15)


def install_webots() -> bool:
    """Run the Webots installation shell script.
    Запускает shell-скрипт установки Webots.
    """
    script = _SCRIPTS_DIR / "install_webots.sh"
    rc = _run_script(script, f"Установка Webots {_WEBOTS_VERSION}")
    return rc in (0, -9, -15)


# Build prerequisites
# Предусловия сборки
def _missing_apt_prereqs() -> list[str]:
    """Return the subset of _APT_PREREQS that is not currently installed via dpkg.
    Возвращает подмножество _APT_PREREQS, которое сейчас не установлено через dpkg.
    """
    missing = []
    for pkg in _APT_PREREQS:
        r = subprocess.run(
            ["dpkg-query", "-W", "-f=${Status}", pkg],
            capture_output=True, text=True,
        )
        if "install ok installed" not in r.stdout:
            missing.append(pkg)
    return missing


def _ensure_root_pip_break(p: process.StepProgress) -> None:
    """Let root's pip override PEP 668, scoped to /root/.config/pip/pip.conf.

    rosdep installs the pip-based rosdep keys (fastapi, uvicorn, multipart, fastmcp)
    as root via sudo; on Ubuntu 24.04 that is blocked by PEP 668 unless break-system-
    packages is allowed. Writing root's pip config is idempotent, reversible (just
    delete the file), and does not touch the user's own pip configuration.

    Разрешает pip от root обходить PEP 668, ограничиваясь /root/.config/pip/pip.conf.
    rosdep ставит pip-зависимости от root через sudo; на Ubuntu 24.04 это блокируется
    PEP 668, пока не разрешён break-system-packages. Запись конфига pip от root
    идемпотентна, обратима (удалить файл) и не трогает пользовательский pip.
    """
    snippet = (
        "mkdir -p /root/.config/pip && "
        "( grep -qs 'break-system-packages' /root/.config/pip/pip.conf || "
        "printf '[global]\\nbreak-system-packages = true\\n' "
        ">> /root/.config/pip/pip.conf )"
    )
    process.stream(privilege.sudo(["bash", "-c", snippet]), on_line=p.log)
    # rosdep calls `pip install -U <pkg>` which tries to upgrade all transitive deps,
    # including packages installed by apt that have no pip RECORD file. Pre-installing
    # them via pip creates the RECORD so the later upgrade can proceed cleanly.
    process.stream(
        privilege.sudo(["pip3", "install", "--break-system-packages",
                        "--ignore-installed", "typing-extensions"]),
        on_line=p.log,
    )


def _register_rosdep_source(p: process.StepProgress, env: dict) -> None:
    """Register the project's local rosdep.yaml as a rosdep source and run rosdep update.
    Only re-writes / updates when the source file is missing or out of date.
    Регистрирует локальный rosdep.yaml проекта как источник rosdep и запускает rosdep update.
    Перезаписывает/обновляет только если файл-источник отсутствует или устарел.
    """
    rosdep_yaml = _PROJECT_DIR / "rosdep.yaml"
    if not rosdep_yaml.exists():
        return
    sources_list = Path("/etc/ros/rosdep/sources.list.d/50-kuka-local.list")
    entry = f"yaml file://{rosdep_yaml}\n"
    try:
        current = sources_list.read_text() if sources_list.exists() else ""
    except Exception:
        current = ""
    if current == entry:
        return
    snippet = (
        "mkdir -p /etc/ros/rosdep/sources.list.d && "
        f"printf '%s\\n' 'yaml file://{rosdep_yaml}' > {sources_list}"
    )
    process.stream(privilege.sudo(["bash", "-c", snippet]), on_line=p.log)
    p.log(f"Зарегистрирован локальный источник rosdep: {rosdep_yaml}")
    process.stream(["rosdep", "update"], env=env, cwd=str(_PROJECT_DIR), on_line=p.log)


def _count_colcon_packages(env: dict) -> int:
    """Count colcon packages under src/ so the build bar can show X / total.
    Считает пакеты colcon в src/, чтобы бар сборки показывал X / всего.
    """
    r = subprocess.run(
        ["colcon", "list", "--base-paths", "src"],
        capture_output=True, text=True, cwd=str(_PROJECT_DIR), env=env,
    )
    return max(len([l for l in r.stdout.splitlines() if l.strip()]), 1)


def build_workspace() -> bool:
    """Build the workspace: apt prerequisites -> rosdep install -> colcon build.

    Step 1 guarantees python3-pip/dev/venv and allows root pip under PEP 668 so the
    pip-based rosdep keys install cleanly. Step 2 runs rosdep install with
    PIP_BREAK_SYSTEM_PACKAGES=1. Step 3 compiles every package with live progress.

    Собирает workspace: apt-предусловия -> rosdep install -> colcon build.
    Шаг 1 гарантирует python3-pip/dev/venv и разрешает pip от root под PEP 668. Шаг 2
    запускает rosdep install с PIP_BREAK_SYSTEM_PACKAGES=1. Шаг 3 компилирует все пакеты.
    """
    env = _ros2_env()
    env["PIP_BREAK_SYSTEM_PACKAGES"] = "1"

    if not shutil.which("colcon") and not Path(f"/opt/ros/{_DISTRO}/bin/colcon").exists():
        header("Сборка проекта")
        ui.error("colcon не найден.")
        ui.note(f"Сначала выполните: source /opt/ros/{_DISTRO}/setup.bash")
        done(False, "colcon недоступен")
        return False

    ok = True
    fail_msg = ""

    with process.StepProgress("Сборка проекта") as p:
        # --- Шаг 1/3: системные зависимости pip (apt) ---
        p.raw("[bold]Шаг 1/3 — системные зависимости pip (apt)[/bold]")
        p.set(0, "Проверка python3-pip / dev / venv...")
        missing = _missing_apt_prereqs()
        if missing:
            p.log(f"Установка: {', '.join(missing)}")
            process.stream(privilege.sudo(["apt-get", "update", "-q"]), env=env, on_line=p.log)
            rc = process.stream(
                privilege.sudo(["apt-get", "install", "-y", *missing]),
                env=env, on_line=p.log,
            )
            if rc not in (0, -9, -15):
                ok, fail_msg = False, "Не удалось установить apt-зависимости"
        else:
            p.log("python3-pip / dev / venv уже установлены")
        if ok:
            _ensure_root_pip_break(p)

        # --- Шаг 2/3: rosdep install ---
        if ok:
            p.set(10, "rosdep install...")
            p.raw("\n[bold]Шаг 2/3 — rosdep install[/bold]")
            _register_rosdep_source(p, env)
            rc = process.stream(
                ["rosdep", "install", "--from-paths", "src", "-i", "-r", "-y"],
                env=env, cwd=str(_PROJECT_DIR), on_line=p.log,
            )
            if rc not in (0, -9, -15):
                ok, fail_msg = False, "rosdep install завершился с ошибкой"

        # --- Шаг 3/3: colcon build ---
        if ok:
            total = _count_colcon_packages(env)
            p.set(30, f"0 / {total} пакетов")
            p.raw(f"\n[bold]Шаг 3/3 — colcon build ({total} пакетов)[/bold]")
            built = 0

            def _on_build(s: str) -> None:
                nonlocal built
                if s:
                    p.log(s)
                if "Finished <<<" in s or "Failed <<<" in s:
                    built += 1
                    p.set(30 + built / total * 70, f"{built} / {total} пакетов")

            rc = process.stream(
                ["colcon", "build", "--base-paths", "src"],
                env=env, cwd=str(_PROJECT_DIR), on_line=_on_build,
            )
            if rc not in (0, -9, -15):
                ok, fail_msg = False, "colcon build завершился с ошибкой"
            else:
                p.set(100, "Готово")

    done(ok, "Сборка завершена" if ok else fail_msg)
    if ok:
        ui.note("Активируйте окружение:  source install/setup.bash")
    return ok


# Interactive flow
# Интерактивный сценарий
def run(args: argparse.Namespace) -> None:
    """Guide the user through installing ROS2 Jazzy and building the workspace.
    Проводит пользователя через установку ROS2 Jazzy и сборку workspace.
    """
    header("Локальная установка", "ROS2 Jazzy + сборка проекта")

    choice = ui.select("Установить ROS2 Jazzy?", ["Да, установить", "Нет, выход"],
                       "Да, установить")
    if not choice or choice.startswith("Нет"):
        return

    # Acquire sudo once, up front, with the masked prompt + keep-alive thread.
    # Получаем sudo один раз, заранее, с маскированным вводом + keep-alive потоком.
    if not privilege.ensure_sudo():
        return

    if not _detect_ubuntu_2404():
        v = ui.select(
            "Ubuntu 24.04 не обнаружена. Настроить окружение через Docker?",
            ["Да, запустить docker-setup", "Нет, выход"],
            "Да, запустить docker-setup",
        )
        if v and v.startswith("Да"):
            _docker_setup(args)
        return

    variant = ui.select(
        "Какой вариант ROS2 Jazzy установить?",
        ["Desktop (полный, с GUI-инструментами)", "Base (минимальный, без GUI)"],
        "Desktop (полный, с GUI-инструментами)",
    )
    if not variant:
        return
    pkg = "desktop" if variant.startswith("Desktop") else "ros-base"
    if not install_ros2(pkg):
        return

    if ui.confirm("Собрать workspace сейчас? (rosdep install + colcon build)", default=True):
        build_workspace()

    if not webots_installed():
        if ui.confirm(f"Установить симулятор Webots {_WEBOTS_VERSION}?", default=False):
            install_webots()


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register the local-setup subcommand with the CLI argument parser.
    Регистрирует подкоманду local-setup в парсере аргументов командной строки.
    """
    p = subparsers.add_parser(
        "local-setup",
        help="Install ROS2 Jazzy natively and build the project with colcon",
    )
    p.set_defaults(func=run)
