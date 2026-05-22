from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path
from typing import Callable, List, Optional

from textual.app import App

from cobot.tui import SCREEN_CSS, LogScreen, PickScreen
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

# Type alias for the callable used to write a line to the TUI log screen.
# Псевдоним типа для функции записи строки в лог TUI.
Write = Callable[[str], None]


# OS and tool detection helpers
# Вспомогательные функции для определения ОС и наличия инструментов
def _detect_ubuntu_2404() -> bool:
    """Return True if the current OS is Ubuntu 24.04 (Noble).

    Reads /etc/os-release and checks the ID and VERSION_ID fields.
    Возвращает True, если текущая ОС - Ubuntu 24.04 (Noble).
    Читает /etc/os-release и проверяет поля ID и VERSION_ID.
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
    Запускает /opt/ros/jazzy/setup.bash в подпроцессе, перехватывает все
    экспортированные переменные и объединяет их с копией os.environ.
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
    return env


# Subprocess runner helpers
# Вспомогательные функции для запуска подпроцессов
def _run_logged(
    cmd: List[str],
    write: Write,
    env: dict | None = None,
    cwd=None,
    register_proc: Callable | None = None,
) -> None:
    """Run a command and stream every non-empty output line to the TUI log.

    Raises RuntimeError if the process exits with a non-zero code (SIGKILL is
    treated as a normal cancellation and does not raise).

    Запускает команду и передаёт каждую непустую строку вывода в лог TUI.
    Выбрасывает RuntimeError если процесс завершился с ненулевым кодом
    (SIGKILL считается нормальной отменой и не вызывает исключение).
    """
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env or os.environ,
        cwd=cwd,
    )
    if register_proc:
        register_proc(proc)
    for line in proc.stdout:
        s = line.rstrip()
        if s:
            write(s)
    proc.wait()
    if proc.returncode not in (0, -9):
        raise RuntimeError(f"Command failed: {cmd[0]}")


# Tasks - long-running functions executed inside a LogScreen background thread
# Задачи - долгие функции, выполняемые в фоновом потоке внутри LogScreen
def _run_script(script: Path, screen: LogScreen) -> None:
    """Run a shell script, stream its output to the TUI log, and parse
    PROGRESS:<pct>:<label> markers to update the progress bar.
    Raises RuntimeError if the script exits with a non-zero code.
    Запускает shell-скрипт, транслирует вывод в лог TUI и разбирает маркеры
    PROGRESS:<pct>:<метка> для обновления прогресс-бара.
    Выбрасывает RuntimeError если скрипт завершился с ненулевым кодом.
    """
    proc = subprocess.Popen(
        ["bash", str(script)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, cwd=_PROJECT_DIR,
    )
    screen.set_proc(proc)
    for line in proc.stdout:
        s = line.rstrip()
        if s.startswith("PROGRESS:"):
            # Format emitted by scripts: PROGRESS:<pct>:<label>
            # Формат, выводимый скриптами: PROGRESS:<pct>:<метка>
            parts = s.split(":", 2)
            try:
                screen.set_progress(float(parts[1]), parts[2] if len(parts) > 2 else "")
            except (ValueError, IndexError):
                pass
        elif s:
            screen.write(s)
    proc.wait()
    if proc.returncode not in (0, -9):
        raise RuntimeError(f"Script failed (exit {proc.returncode}): {script.name}")


def _task_install(screen: LogScreen, pkg: str) -> None:
    """Run the ROS2 Jazzy installation shell script for the chosen variant (desktop / ros-base).
    The script emits PROGRESS: markers so the bar advances during installation.
    Запускает shell-скрипт установки ROS2 Jazzy для выбранного варианта (desktop / ros-base).
    Скрипт выводит маркеры PROGRESS:, чтобы прогресс-бар обновлялся во время установки.
    """
    try:
        # "desktop" -> setup_ros2_desktop.sh, "ros-base" -> setup_ros2_ros_base.sh
        script = _SCRIPTS_DIR / f"setup_ros2_{pkg.replace('-', '_')}.sh"
        if not script.exists():
            screen.write(f"[red]Script not found:[/red] {script}")
            screen.finish(False)
            return
        screen.set_progress(0, "Starting installation...")
        _run_script(script, screen)
        if not screen.is_stopped():
            screen.set_progress(100, "Done")
            screen.write(f"\n[green]ROS2 {_DISTRO} ({pkg}) installed successfully.[/green]")
            screen.finish(True)
    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


def _task_build(screen: LogScreen) -> None:
    """Build the project workspace using rosdep and colcon.

    Step 1 - runs "rosdep install --from-paths src" to pull in all package
    dependencies declared in the src/ directory.
    Step 2 - runs "colcon build --symlink-install" to compile every package.

    Both commands receive a copy of os.environ extended with the sourced ROS2
    setup so that ament CMake macros and ROS2 packages are visible even if the
    user has not yet sourced setup.bash in this terminal session.

    Собирает рабочее пространство проекта с помощью rosdep и colcon.
    Шаг 1 - запускает "rosdep install --from-paths src" для установки всех
    зависимостей пакетов, объявленных в директории src/.
    Шаг 2 - запускает "colcon build --symlink-install" для компиляции каждого пакета.

    Обе команды получают копию os.environ с подключённым окружением ROS2, так что
    макросы ament CMake и пакеты ROS2 видны даже если пользователь ещё не выполнил
    source setup.bash в этой сессии терминала.
    """
    try:
        env = _ros2_env()

        if not shutil.which("colcon") and not Path(f"/opt/ros/{_DISTRO}/bin/colcon").exists():
            screen.write("[red]colcon not found.[/red]")
            screen.write(f"Source ROS2 first:  [bold]source /opt/ros/{_DISTRO}/setup.bash[/bold]")
            screen.finish(False)
            return

        screen.set_progress(0, "Installing dependencies...")
        screen.write("[bold]Step 1 / 2 - rosdep install[/bold]\n")
        _run_logged(
            ["rosdep", "install", "--from-paths", "src", "-i", "-r", "-y"],
            screen.write,
            env=env,
            cwd=_PROJECT_DIR,
            register_proc=screen.set_proc,
        )
        if screen.is_stopped():
            return

        screen.set_progress(30, "Building...")
        list_result = subprocess.run(
            ["colcon", "list"], capture_output=True, text=True,
            cwd=_PROJECT_DIR, env=env,
        )
        total = max(len([l for l in list_result.stdout.splitlines() if l.strip()]), 1)
        screen.write(f"\n[bold]Step 2 / 2 - colcon build ({total} packages)[/bold]\n")
        built = 0

        def _track(line: str) -> None:
            """Update the progress bar each time colcon finishes a package.
            Обновляет прогресс-бар каждый раз, когда colcon завершает пакет.
            """
            nonlocal built
            screen.write(line)
            if "Finished <<<" in line or "Failed <<<" in line:
                built += 1
                screen.set_progress(
                    30 + built / total * 70,
                    f"{built} / {total} packages done",
                )

        _run_logged(
            ["colcon", "build"],
            _track,
            env=env,
            cwd=_PROJECT_DIR,
            register_proc=screen.set_proc,
        )

        if not screen.is_stopped():
            screen.set_progress(100, "Build complete")
            screen.write("\nActivate workspace:  [bold]source install/setup.bash[/bold]")
            screen.finish(True)
    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


def _task_install_webots(screen: LogScreen) -> None:
    """Run the Webots installation shell script, streaming output and progress to the TUI.

    Запускает shell-скрипт установки Webots, транслируя вывод и прогресс в TUI.
    """
    try:
        script = _SCRIPTS_DIR / "install_webots.sh"
        if not script.exists():
            screen.write(f"[red]Script not found:[/red] {script}")
            screen.finish(False)
            return
        screen.set_progress(0, "Starting Webots installation...")
        _run_script(script, screen)
        if not screen.is_stopped():
            screen.set_progress(100, "Done")
            screen.write(f"\n[green]Webots {_WEBOTS_VERSION} installed successfully.[/green]")
            screen.finish(True)
    except Exception as exc:
        if not screen.is_stopped():
            screen.write(f"\n[red]Error:[/red] {exc}")
            screen.finish(False)


# TUI application - orchestrates screens and user choices
# TUI приложение - управляет экранами и выборами пользователя
class _LocalSetupApp(App[Optional[str]]):
    """Main TUI application for the local-setup command.

    Guides the user through: install ROS2 choice, OS check, version choice,
    installation log, build log, and optional Webots installation.
    Returns "docker" if the user opts for Docker setup, None otherwise.

    Главное TUI приложение для команды local-setup.
    Проводит пользователя через: выбор установки ROS2, проверку ОС, выбор версии,
    лог установки, лог сборки и опциональную установку Webots.
    Возвращает "docker" если пользователь выбирает Docker, иначе None.
    """

    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(
                "local-setup",
                "Install ROS2 Jazzy?",
                ["Yes, install", "No, exit"],
                "Yes, install",
            ),
            self._on_install_choice,
        )

    def _on_install_choice(self, choice: Optional[str]) -> None:
        """Handle the initial yes/no choice to install ROS2.
        Обрабатывает начальный выбор да/нет для установки ROS2.
        """
        if not choice or choice.startswith("No"):
            self.exit(None)
            return
        if not _detect_ubuntu_2404():
            self.push_screen(
                PickScreen(
                    "Unsupported OS",
                    "Ubuntu 24.04 not detected. Set up the environment via Docker instead?",
                    ["Yes, run docker-setup", "No, exit"],
                    "Yes, run docker-setup",
                ),
                self._on_docker_choice,
            )
        else:
            self.push_screen(
                PickScreen(
                    "ROS2 version",
                    "Which ROS2 Jazzy variant do you want to install?",
                    ["Desktop (full install, includes GUI tools)", "Base (minimal, no GUI)"],
                    "Desktop (full install, includes GUI tools)",
                ),
                self._on_version_choice,
            )

    def _on_docker_choice(self, choice: Optional[str]) -> None:
        """Exit the app signalling whether docker-setup should be launched.
        Завершает приложение, сигнализируя нужно ли запустить docker-setup.
        """
        self.exit("docker" if choice and choice.startswith("Yes") else None)

    def _on_version_choice(self, choice: Optional[str]) -> None:
        """Start the installation log screen for the chosen ROS2 variant.
        Запускает экран лога установки для выбранного варианта ROS2.
        """
        if not choice:
            self.exit(None)
            return
        pkg = "desktop" if choice.startswith("Desktop") else "ros-base"
        self.push_screen(
            LogScreen(
                f"Installing ROS2 Jazzy ({pkg})",
                lambda s: _task_install(s, pkg),
                show_progress=True,
            ),
            lambda _: self._after_install(),
        )

    def _after_install(self) -> None:
        """After installation, ask whether to build the project workspace now.
        После установки спрашивает, нужно ли собрать рабочее пространство прямо сейчас.
        """
        self.push_screen(
            PickScreen(
                "Build",
                "Build the project workspace now?\n(runs rosdep install + colcon build)",
                ["Yes, build now", "No, skip"],
                "Yes, build now",
            ),
            self._on_build_choice,
        )

    def _on_build_choice(self, choice: Optional[str]) -> None:
        """Start the build log screen or skip directly to the Webots prompt.
        Запускает экран сборки или пропускает к вопросу про Webots.
        """
        if not choice or choice.startswith("No"):
            self._after_build()
            return
        self.push_screen(
            LogScreen("Building project", _task_build, show_progress=True),
            lambda _: self._after_build(),
        )

    def _after_build(self) -> None:
        """After the build, offer to install Webots if it is not already present.
        После сборки предлагает установить Webots если он ещё не установлен.
        """
        if webots_installed():
            self.exit(None)
            return
        self.push_screen(
            PickScreen(
                "Webots",
                f"Install Webots {_WEBOTS_VERSION} simulator?",
                [f"Yes, install Webots {_WEBOTS_VERSION}", "No, skip"],
                "No, skip",
            ),
            self._on_webots_choice,
        )

    def _on_webots_choice(self, choice: Optional[str]) -> None:
        """Start the Webots installer or exit depending on the user choice.
        Запускает установщик Webots или завершает работу в зависимости от выбора.
        """
        if choice and choice.startswith("Yes"):
            subprocess.run(["sudo", "-v"], check=False)
            self.push_screen(
                LogScreen(
                    f"Installing Webots {_WEBOTS_VERSION}",
                    _task_install_webots,
                    show_progress=True,
                ),
                lambda _: self.exit(None),
            )
        else:
            self.exit(None)


class WebotsInstallApp(App[bool]):
    """Standalone TUI app for installing Webots, used by the run command.

    Launched by run.py when the user starts a local simulation but Webots
    is not installed yet.

    Отдельное TUI приложение для установки Webots, используемое командой run.
    Запускается из run.py когда пользователь запускает локальную симуляцию,
    но Webots ещё не установлен.
    """

    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            LogScreen(
                f"Installing Webots {_WEBOTS_VERSION}",
                _task_install_webots,
                show_progress=True,
            ),
            self.exit,
        )


# Entry point - registered as the "local-setup" subcommand
# Точка входа - зарегистрирована как подкоманда "local-setup"
def register(subparsers: argparse._SubParsersAction) -> None:
    """Register the local-setup subcommand with the CLI argument parser.

    Регистрирует подкоманду local-setup в парсере аргументов командной строки.
    """
    p = subparsers.add_parser(
        "local-setup",
        help="Install ROS2 Jazzy natively and build the project with colcon",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    """Entry point for the local-setup command.

    Pre-caches the sudo token while the terminal is in normal mode so that
    subsequent sudo calls inside the Textual TUI do not hang waiting for
    a password prompt that the user cannot see.

    Точка входа для команды local-setup.
    Предварительно кеширует sudo-токен пока терминал в обычном режиме, чтобы
    последующие вызовы sudo внутри Textual TUI не зависали ожидая запрос пароля,
    который пользователь не может увидеть.
    """
    subprocess.run(["sudo", "-v"], check=False)
    result = _LocalSetupApp().run()
    if result == "docker":
        _docker_setup(args)
