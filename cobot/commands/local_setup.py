from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
import threading
import urllib.request
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

# Path where apt expects the ROS2 GPG signing key to be stored.
# Путь, по которому apt ожидает найти GPG-ключ подписи ROS2.
_ROS_KEYRING = Path("/usr/share/keyrings/ros-archive-keyring.gpg")

# Path to the apt sources file that points to the ROS2 package repository.
# Путь к файлу источников apt, указывающему на репозиторий пакетов ROS2.
_ROS_SOURCES = Path("/etc/apt/sources.list.d/ros2.list")

# Path created by "rosdep init" to mark that rosdep has already been initialized.
# Путь, создаваемый "rosdep init" для отметки того, что rosdep уже инициализирован.
_ROSDEP_SOURCES = Path("/etc/ros/rosdep/sources.list.d/20-default.list")

# Webots simulator version and the direct .deb download URL for amd64.
# Версия симулятора Webots и прямая ссылка для скачивания .deb для amd64.
_WEBOTS_VERSION = "2025a"
_WEBOTS_DEB_URL = (
    f"https://github.com/cyberbotics/webots/releases/download/"
    f"R{_WEBOTS_VERSION}/webots_{_WEBOTS_VERSION}_amd64.deb"
)

# Environment variables passed to apt to suppress interactive prompts (e.g. "restart services?").
# Переменные окружения для apt, подавляющие интерактивные запросы (например, "перезапустить сервисы?").
_APT_ENV = {**os.environ, "DEBIAN_FRONTEND": "noninteractive"}

# Extra apt options: short per-connection and per-transfer timeouts plus retry count.
# Forces IPv4 because many VMs have broken IPv6 routing that causes silent hangs.
# Дополнительные опции apt: короткие таймауты на соединение и передачу данных, плюс число повторов.
# Принудительно используем IPv4, так как в VM часто сломана маршрутизация IPv6, что вызывает зависания.
_APT_OPTS = [
    "-o", "Acquire::http::ConnectTimeout=15",
    "-o", "Acquire::https::ConnectTimeout=15",
    "-o", "Acquire::http::Timeout=30",
    "-o", "Acquire::https::Timeout=30",
    "-o", "Acquire::Retries=2",
    "-o", "Acquire::ForceIPv4=true",
]

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


def _run_apt_with_progress(
    cmd: List[str],
    write: Write,
    on_progress: Callable[[float], None],
    env: dict | None = None,
    register_proc: Callable | None = None,
) -> None:
    """Run an apt command, stream its stdout to the log, and report download progress.

    Uses APT::Status-Fd to receive machine-readable progress lines on a private
    pipe. A background thread reads the pipe and calls on_progress(0-100) for
    each percentage update. When a new URI starts downloading it is printed to
    the log so the user can see what is being fetched.

    Запускает команду apt, передаёт stdout в лог и показывает прогресс скачивания.
    Использует APT::Status-Fd для получения машинночитаемых строк прогресса через
    приватный канал. Фоновый поток читает канал и вызывает on_progress(0-100) при
    каждом обновлении процента. При начале скачивания нового файла его URI
    выводится в лог, чтобы пользователь видел что именно загружается.
    """
    r_fd, w_fd = os.pipe()
    try:
        proc = subprocess.Popen(
            cmd + ["-o", f"APT::Status-Fd={w_fd}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env or os.environ,
            pass_fds=(w_fd,),
        )
    finally:
        # Close the write end in the parent so the reader thread gets EOF when apt exits.
        # Закрываем пишущий конец в родителе, чтобы читающий поток получил EOF при выходе apt.
        os.close(w_fd)

    if register_proc:
        register_proc(proc)

    def _read_status() -> None:
        """Parse APT::Status-Fd lines and forward percentage and URI info.

        Status-Fd line format: dlstatus:index:pct:message
        or for package installs: pmstatus:pkg:pct:message

        Разбирает строки APT::Status-Fd и передаёт процент и URI.
        Формат строки: dlstatus:index:pct:message
        или для установки пакетов: pmstatus:pkg:pct:message
        """
        last_uri: str = ""
        fetched = 0
        with os.fdopen(r_fd, "r") as f:
            for line in f:
                parts = line.strip().split(":", 3)
                if len(parts) < 3:
                    continue
                kind, _, pct_str = parts[0], parts[1], parts[2]
                msg = parts[3] if len(parts) == 4 else ""
                try:
                    on_progress(float(pct_str))
                except ValueError:
                    continue
                if kind == "dlstatus" and msg:
                    # Print each new URI once as it starts downloading.
                    # Выводим каждый новый URI один раз при начале загрузки.
                    uri = msg.split()[0]
                    if uri != last_uri:
                        last_uri = uri
                        fetched += 1
                        write(f"[dim][{fetched}] {msg}[/dim]")

    t = threading.Thread(target=_read_status, daemon=True)
    t.start()
    for line in proc.stdout:
        s = line.rstrip()
        if s:
            write(s)
    proc.wait()
    t.join()

    if proc.returncode not in (0, -9):
        raise RuntimeError(f"Command failed: {cmd[0]}")


# Installation steps - each step maps to one visible phase in the log screen
# Шаги установки - каждый шаг соответствует одной видимой фазе в экране лога
def _step_prereqs(
    write: Write,
    on_progress: Callable[[float], None],
    register_proc: Callable | None = None,
) -> None:
    """Step 1 - Refresh apt cache and install packages required for the ROS2 setup.

    Installs: software-properties-common, curl, gnupg2, lsb-release, build-essential.
    Also enables the Ubuntu universe repository which some ROS2 dependencies live in.

    Шаг 1 - Обновляет кеш apt и устанавливает пакеты, необходимые для настройки ROS2.
    Устанавливает: software-properties-common, curl, gnupg2, lsb-release, build-essential.
    Также включает репозиторий Ubuntu universe, в котором находятся некоторые зависимости ROS2.
    """
    write("[bold]Step 1 / 5 - Prerequisites[/bold]")
    write("[cyan][*][/cyan] Updating package lists...")
    _run_apt_with_progress(
        ["sudo", "apt-get", "update"] + _APT_OPTS,
        write, on_progress, _APT_ENV, register_proc,
    )
    write("[cyan][*][/cyan] Installing prerequisites...")
    _run_apt_with_progress(
        [
            "sudo", "apt-get", "install", "-y", "--no-install-recommends",
            "software-properties-common", "curl", "gnupg2",
            "lsb-release", "build-essential",
        ] + _APT_OPTS,
        write, on_progress, _APT_ENV, register_proc,
    )
    write("[cyan][*][/cyan] Adding universe repository...")
    # --no-update prevents add-apt-repository from running its own apt-get update,
    # which would ignore our timeout options and could hang indefinitely.
    # --no-update запрещает add-apt-repository запускать собственный apt-get update,
    # который игнорирует наши таймауты и может зависнуть.
    _run_logged(["sudo", "add-apt-repository", "-y", "--no-update", "universe"], write,
                register_proc=register_proc)
    write("[green][ok][/green] Prerequisites ready")


def _step_ros2_repo(
    write: Write,
    on_progress: Callable[[float], None],
    register_proc: Callable | None = None,
) -> None:
    """Step 2 - Download the ROS2 signing key and register the ROS2 apt repository.

    Removes any previous key and sources file first so re-runs always start clean.
    Downloads the key from the official ros/rosdistro GitHub repository, writes it
    to the system keyring, then creates /etc/apt/sources.list.d/ros2.list and
    refreshes only that source to avoid updating all Ubuntu mirrors.

    Шаг 2 - Скачивает ключ подписи ROS2 и регистрирует репозиторий apt ROS2.
    Сначала удаляет предыдущие ключ и файл источников, чтобы повторные запуски
    всегда начинались с чистого состояния. Скачивает ключ с официального GitHub
    репозитория ros/rosdistro, записывает его в системный кейринг, затем создаёт
    /etc/apt/sources.list.d/ros2.list и обновляет только этот источник.
    """
    write("\n[bold]Step 2 / 5 - ROS2 repository[/bold]")

    # Remove previous key and sources file to avoid stale config on re-runs.
    # Удаляем предыдущие ключ и файл источников для чистого состояния при повторных запусках.
    for path in (_ROS_KEYRING, _ROS_SOURCES):
        if path.exists():
            subprocess.run(["sudo", "rm", "-f", str(path)], check=False)
            write(f"[dim]Removed {path}[/dim]")

    write("[cyan][*][/cyan] Downloading ROS2 signing key...")
    key_url = "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
    with tempfile.NamedTemporaryFile(delete=False, suffix=".gpg") as tmp:
        tmp_path = tmp.name
    try:
        with urllib.request.urlopen(key_url, timeout=30) as resp:
            Path(tmp_path).write_bytes(resp.read())
        subprocess.run(
            ["sudo", "install", "-m", "644", tmp_path, str(_ROS_KEYRING)],
            check=True,
        )
    finally:
        Path(tmp_path).unlink(missing_ok=True)
    write("[green][ok][/green] Signing key installed")

    # Detect machine architecture and Ubuntu codename to build the sources.list line.
    # Определяем архитектуру машины и кодовое имя Ubuntu для строки sources.list.
    arch = subprocess.check_output(["dpkg", "--print-architecture"], text=True).strip()
    codename = subprocess.check_output(
        ["bash", "-c", ". /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}}"],
        text=True,
    ).strip()
    sources_line = (
        f"deb [arch={arch} signed-by={_ROS_KEYRING}] "
        f"http://packages.ros.org/ros2/ubuntu {codename} main\n"
    )
    result = subprocess.run(
        ["sudo", "tee", str(_ROS_SOURCES)],
        input=sources_line, capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Failed to write {_ROS_SOURCES}")
    write(f"[dim]{_ROS_SOURCES}[/dim]")

    # Update only the ROS2 source - avoids downloading all Ubuntu mirror metadata.
    # Обновляем только источник ROS2 - избегаем скачивания метаданных всех зеркал Ubuntu.
    write("[cyan][*][/cyan] Updating ROS2 package list...")
    _run_apt_with_progress(
        [
            "sudo", "apt-get", "update",
            "-o", f"Dir::Etc::sourcelist={_ROS_SOURCES}",
            "-o", "Dir::Etc::sourceparts=-",
            "-o", "APT::Get::List-Cleanup=0",
        ] + _APT_OPTS,
        write, on_progress, _APT_ENV, register_proc,
    )
    write("[green][ok][/green] ROS2 repository ready")


def _step_install_ros2(
    write: Write,
    on_progress: Callable[[float], None],
    pkg: str,
    register_proc: Callable | None = None,
) -> None:
    """Step 3 - Install the chosen ROS2 Jazzy package via apt.

    pkg is either "desktop" (full install with GUI tools) or "ros-base"
    (minimal headless install). The full package name becomes ros-jazzy-{pkg}.

    Шаг 3 - Устанавливает выбранный пакет ROS2 Jazzy через apt.
    pkg - это либо "desktop" (полная установка с GUI), либо "ros-base"
    (минимальная установка без GUI). Полное имя пакета: ros-jazzy-{pkg}.
    """
    write(f"\n[bold]Step 3 / 5 - ros-{_DISTRO}-{pkg}[/bold]")
    write(f"[cyan][*][/cyan] Installing ros-{_DISTRO}-{pkg}...")
    _run_apt_with_progress(
        ["sudo", "apt-get", "install", "-y", f"ros-{_DISTRO}-{pkg}"] + _APT_OPTS,
        write, on_progress, _APT_ENV, register_proc,
    )
    write(f"[green][ok][/green] ros-{_DISTRO}-{pkg} installed")


def _step_dev_tools(
    write: Write,
    on_progress: Callable[[float], None],
    register_proc: Callable | None = None,
) -> None:
    """Step 4 - Install colcon, rosdep, vcstool and initialize rosdep.

    Installs the Python packages needed to build and manage ROS2 workspaces.
    Runs "rosdep init" only if it has not been run before, then always runs
    "rosdep update" to fetch the latest package index.

    Шаг 4 - Устанавливает colcon, rosdep, vcstool и инициализирует rosdep.
    Устанавливает Python-пакеты, необходимые для сборки и управления рабочими
    пространствами ROS2. Запускает "rosdep init" только если он ещё не запускался,
    затем всегда запускает "rosdep update" для получения свежего индекса пакетов.
    """
    write("\n[bold]Step 4 / 5 - Dev tools[/bold]")
    write("[cyan][*][/cyan] Installing colcon, rosdep, vcstool...")
    _run_apt_with_progress(
        [
            "sudo", "apt-get", "install", "-y",
            "python3-argcomplete",
            "python3-colcon-clean",
            "python3-colcon-common-extensions",
            "python3-rosdep",
            "python3-vcstool",
        ] + _APT_OPTS,
        write, on_progress, _APT_ENV, register_proc,
    )
    if not _ROSDEP_SOURCES.exists():
        write("[cyan][*][/cyan] Initializing rosdep...")
        _run_logged(["sudo", "rosdep", "init"], write, register_proc=register_proc)
    else:
        write("[green][ok][/green] rosdep already initialized")
    write("[cyan][*][/cyan] Updating rosdep...")
    _run_logged(["rosdep", "update", "--rosdistro", _DISTRO], write,
                register_proc=register_proc)
    write("[green][ok][/green] Dev tools ready")


def _step_shell_setup(write: Write) -> None:
    """Step 5 - Append ROS2 environment setup lines to the user shell rc file.

    Adds "source /opt/ros/jazzy/setup.bash" so ROS2 commands are available in
    every new terminal. Also adds a commented-out ROS_AUTOMATIC_DISCOVERY_RANGE
    line as a reminder for multi-machine setups. Both lines are added only once.

    Шаг 5 - Добавляет строки настройки окружения ROS2 в rc-файл оболочки пользователя.
    Добавляет "source /opt/ros/jazzy/setup.bash" чтобы команды ROS2 были доступны
    в каждом новом терминале. Также добавляет закомментированную строку
    ROS_AUTOMATIC_DISCOVERY_RANGE как напоминание для многомашинных настроек.
    Обе строки добавляются только один раз.
    """
    write("\n[bold]Step 5 / 5 - Shell configuration[/bold]")
    shell_name = Path(os.environ.get("SHELL", "/bin/bash")).name
    rc = Path.home() / (".zshrc" if shell_name == "zsh" else ".bashrc")
    rc_text = rc.read_text() if rc.exists() else ""

    source_line = f"source /opt/ros/{_DISTRO}/setup.bash"
    discovery_comment = "# export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"

    additions = []
    if source_line not in rc_text:
        additions.append(source_line)
    if discovery_comment not in rc_text:
        additions.append(discovery_comment)

    if additions:
        with rc.open("a") as f:
            f.write(f"\n# ROS2 {_DISTRO}\n")
            for line in additions:
                f.write(line + "\n")
        write(f"[green][ok][/green] Added ROS2 setup to ~/{rc.name}")
    else:
        write(f"[green][ok][/green] ROS2 setup already in ~/{rc.name}")


# Tasks - long-running functions executed inside a LogScreen background thread
# Задачи - долгие функции, выполняемые в фоновом потоке внутри LogScreen
def _task_install(screen: LogScreen, pkg: str) -> None:
    """Full ROS2 Jazzy installation task, split into 5 sequential steps.

    Maps each step to a sub-range of the 0-100% progress bar so the bar
    advances smoothly through prerequisites, repo setup, ROS2 install,
    dev tools and shell configuration.

    Полная задача установки ROS2 Jazzy, разбитая на 5 последовательных шагов.
    Каждый шаг отображается в своём диапазоне прогресс-бара 0-100%, так что
    бар плавно движется через prerequisites, настройку репозитория, установку
    ROS2, инструменты разработчика и настройку оболочки.
    """
    try:
        def prog(lo: float, hi: float) -> Callable[[float], None]:
            """Map a 0-100 apt percentage into the [lo, hi] sub-range of the progress bar.
            Отображает 0-100% apt в поддиапазон [lo, hi] прогресс-бара.
            """
            return lambda p: screen.set_progress(lo + p / 100.0 * (hi - lo))

        screen.set_progress(0, "Preparing...")
        _step_prereqs(screen.write, prog(0, 15), register_proc=screen.set_proc)
        if screen.is_stopped():
            return

        screen.set_progress(15, "Setting up ROS2 repository...")
        _step_ros2_repo(screen.write, prog(15, 30), register_proc=screen.set_proc)
        if screen.is_stopped():
            return

        screen.set_progress(30, f"Installing ros-{_DISTRO}-{pkg}...")
        _step_install_ros2(screen.write, prog(30, 75), pkg, register_proc=screen.set_proc)
        if screen.is_stopped():
            return

        screen.set_progress(75, "Installing dev tools...")
        _step_dev_tools(screen.write, prog(75, 95), register_proc=screen.set_proc)
        if screen.is_stopped():
            return

        screen.set_progress(95, "Configuring shell...")
        _step_shell_setup(screen.write)
        screen.set_progress(100, "Done")

        screen.write(f"\n[green]ROS2 {_DISTRO} installed successfully.[/green]")
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
            ["colcon", "build", "--symlink-install"],
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
    """Download the Webots .deb from GitHub and install it with apt.

    Progress is split into two phases:
    - 0-65%: downloading the .deb file (streamed in 64 KB chunks).
    - 65-100%: running apt-get install on the downloaded file.

    Скачивает .deb Webots с GitHub и устанавливает его через apt.
    Прогресс разделён на две фазы:
    - 0-65%: скачивание .deb файла (потоковое, кусками по 64 КБ).
    - 65-100%: запуск apt-get install для скачанного файла.
    """
    try:
        screen.write(f"[bold]Installing Webots {_WEBOTS_VERSION}[/bold]\n")
        screen.write(f"[dim]{_WEBOTS_DEB_URL}[/dim]\n")
        screen.set_progress(0, "Downloading Webots...")

        with tempfile.TemporaryDirectory() as tmp:
            deb_path = Path(tmp) / f"webots_{_WEBOTS_VERSION}_amd64.deb"

            with urllib.request.urlopen(_WEBOTS_DEB_URL, timeout=30) as resp:
                total = int(resp.headers.get("Content-Length", 0))
                downloaded = 0
                with open(deb_path, "wb") as f:
                    while True:
                        if screen.is_stopped():
                            return
                        chunk = resp.read(65536)
                        if not chunk:
                            break
                        f.write(chunk)
                        downloaded += len(chunk)
                        if total > 0:
                            pct = min(downloaded / total * 65, 65)
                            mb = downloaded / 1_048_576
                            total_mb = total / 1_048_576
                            screen.set_progress(pct, f"Downloading... {mb:.0f} / {total_mb:.0f} MB")

            if screen.is_stopped():
                return

            screen.write("[green]Download complete.[/green]")
            screen.set_progress(65, "Installing package...")

            proc = subprocess.Popen(
                ["sudo", "apt-get", "install", "-y", str(deb_path)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True,
            )
            screen.set_proc(proc)
            for line in proc.stdout:
                s = line.rstrip()
                if s:
                    screen.write(s)
            proc.wait()

        if proc.returncode not in (0, -9):
            screen.write("\n[red]Webots installation failed.[/red]")
            screen.finish(False)
            return

        if screen.is_stopped():
            return

        screen.set_progress(100, "Done")
        screen.write("\n[green]Webots installed successfully.[/green]")
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
        """After installation finishes, immediately start the project build.
        После завершения установки сразу запускает сборку проекта.
        """
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
