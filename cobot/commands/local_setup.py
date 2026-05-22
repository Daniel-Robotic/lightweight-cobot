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

_PROJECT_DIR = Path(__file__).parent.parent.parent

# Paths used for the ROS2 apt repository signing key and sources list.
# Пути для ключа подписи apt-репозитория ROS2 и файла sources list.
_ROS_KEYRING = Path("/usr/share/keyrings/ros-archive-keyring.gpg")
_ROS_SOURCES = Path("/etc/apt/sources.list.d/ros2.list")
_ROS_KEY_URL = "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"

# Suppress apt interactive prompts such as "restart services?".
# Подавляем интерактивные запросы apt, например "перезапустить службы?".
_APT_ENV = {**os.environ, "DEBIAN_FRONTEND": "noninteractive"}

# HTTP/HTTPS timeouts for apt so a stalled server does not hang the process forever.
# 60 seconds per connection attempt is generous enough for any healthy mirror.
# HTTP/HTTPS таймауты для apt, чтобы зависший сервер не блокировал процесс бесконечно.
# 60 секунд на попытку подключения достаточно для любого нормального зеркала.
_APT_TIMEOUTS = [
    "-o", "Acquire::http::Timeout=60",
    "-o", "Acquire::https::Timeout=60",
    "-o", "Acquire::Retries=3",
]


# Check whether we are running on Ubuntu 24.04, which is required for ROS2 Jazzy.
# Проверяем, запущены ли мы на Ubuntu 24.04, которая требуется для ROS2 Jazzy.
def _detect_ubuntu_2404() -> bool:
    path = Path("/etc/os-release")
    if not path.exists():
        return False
    info: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            info[k.strip()] = v.strip().strip('"')
    return info.get("ID") == "ubuntu" and info.get("VERSION_ID") == "24.04"


# Check whether ROS2 Jazzy is already installed by looking for its directory.
# Проверяем, установлен ли ROS2 Jazzy, проверяя наличие его директории.
def _detect_ros2_jazzy() -> bool:
    return Path("/opt/ros/jazzy").is_dir()


Write = Callable[[str], None]


# Run a command and capture output. Print it to the log only if the command fails.
# Запускаем команду и перехватываем вывод. Выводим в лог только если команда завершилась с ошибкой.
def _run_quiet(cmd: List[str], write: Write | None = None, env: dict | None = None, cwd=None) -> None:
    result = subprocess.run(
        cmd, capture_output=True, text=True,
        env=env or os.environ, cwd=cwd,
    )
    if result.returncode != 0:
        if write:
            for line in (result.stdout + result.stderr).splitlines():
                if line.strip():
                    write(line)
        raise RuntimeError(f"Command failed: {cmd[0]}")


# Run a command and stream every output line to the log in real time.
# Запускаем команду и транслируем каждую строку вывода в лог в реальном времени.
def _run_logged(cmd: List[str], write: Write, env: dict | None = None, cwd=None) -> None:
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env or os.environ,
        cwd=cwd,
    )
    for line in proc.stdout:
        s = line.rstrip()
        if s:
            write(s)
    proc.wait()
    if proc.returncode != 0:
        raise RuntimeError(f"Command failed: {cmd[0]}")


def _run_apt_with_progress(
    cmd: List[str],
    write: Write,
    on_progress: Callable[[float], None],
    env: dict | None = None,
) -> None:
    """Run an apt command and feed real percentage from APT::Status-Fd to on_progress(0-100)."""
    # APT::Status-Fd makes apt write progress lines to a pipe descriptor instead of stdout.
    # We read that pipe in a background thread so we can update the progress bar live.
    # APT::Status-Fd заставляет apt писать строки прогресса в дескриптор канала, а не в stdout.
    # Читаем этот канал в фоновом потоке, чтобы обновлять прогресс-бар в реальном времени.
    r_fd, w_fd = os.pipe()
    try:
        proc = subprocess.Popen(
            cmd + [f"-o", f"APT::Status-Fd={w_fd}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env or os.environ,
            pass_fds=(w_fd,),
        )
    finally:
        # Close the write end in the parent process so the reader thread gets EOF when apt exits.
        # Закрываем пишущий конец в родительском процессе, чтобы читающий поток получил EOF при выходе apt.
        os.close(w_fd)

    def _read_status() -> None:
        with os.fdopen(r_fd, "r") as f:
            for line in f:
                # Format: dlstatus:N:PCT:MSG  or  pmstatus:NAME:PCT:MSG
                parts = line.strip().split(":", 3)
                if len(parts) >= 3:
                    try:
                        on_progress(float(parts[2]))
                    except ValueError:
                        pass

    t = threading.Thread(target=_read_status, daemon=True)
    t.start()
    for line in proc.stdout:
        s = line.rstrip()
        if s:
            write(s)
    proc.wait()
    t.join()
    if proc.returncode != 0:
        raise RuntimeError(f"Command failed: {cmd[0]}")


# Make sure the system has a UTF-8 locale, which ROS2 requires to work correctly.
# Убеждаемся, что в системе есть локаль UTF-8, которая требуется ROS2 для корректной работы.
def _setup_locale(write: Write) -> None:
    write("[cyan][*][/cyan] Checking locale...")
    if "UTF-8" in subprocess.run(["locale"], capture_output=True, text=True).stdout:
        write("[green][ok][/green] UTF-8 locale active")
        return
    write("[cyan][*][/cyan] Configuring UTF-8 locale...")
    _run_quiet(["sudo", "apt-get", "update", "-qq"], write)
    _run_quiet(["sudo", "apt-get", "install", "-y", "--no-install-recommends", "locales"], write, _APT_ENV)
    _run_quiet(["sudo", "locale-gen", "en_US.UTF-8"], write)
    _run_quiet(["sudo", "update-locale", "LC_ALL=en_US.UTF-8", "LANG=en_US.UTF-8"], write)
    write("[green][ok][/green] Locale configured")


# Add the official ROS2 apt repository and its signing key so we can install ROS2 packages.
# Добавляем официальный apt-репозиторий ROS2 и его ключ подписи, чтобы можно было установить пакеты ROS2.
def _add_ros2_repo(write: Write, on_progress: Optional[Callable[[float], None]] = None) -> None:
    def _prog(p: float) -> None:
        if on_progress:
            on_progress(p)

    write("[cyan][*][/cyan] Adding ROS2 apt repository...")

    # Best-effort update - 60 second timeout so a bad mirror doesn't hang forever.
    # Фоновое обновление с таймаутом 60 секунд, чтобы зависший зеркальный сервер не блокировал процесс.
    # subprocess.run(["sudo", "apt-get", "update", "-qq"] + _APT_TIMEOUTS, capture_output=True, timeout=120)
    subprocess.run(["sudo", "apt-get", "update"] + _APT_TIMEOUTS, capture_output=True, timeout=120)
    _prog(15)

    _run_quiet(
        ["sudo", "apt-get", "install", "-y", "--no-install-recommends",
         "software-properties-common", "curl", "gnupg"],
        write, _APT_ENV,
    )
    _prog(35)
    _run_quiet(["sudo", "add-apt-repository", "-y", "universe"], write)
    _prog(45)

    write("[cyan][*][/cyan] Downloading ROS2 signing key...")
    with tempfile.NamedTemporaryFile(delete=False, suffix=".key") as tmp:
        tmp_path = tmp.name
    try:
        # 30 second timeout - if GitHub is unreachable we fail fast instead of hanging forever.
        # Таймаут 30 секунд - если GitHub недоступен, падаем быстро вместо бесконечного зависания.
        with urllib.request.urlopen(_ROS_KEY_URL, timeout=30) as resp:
            with open(tmp_path, "wb") as f:
                f.write(resp.read())
        _prog(60)
        # Convert the ASCII-armored key to binary GPG format that apt understands.
        # Конвертируем ключ из ASCII-armor формата в бинарный GPG, который понимает apt.
        _run_quiet(["sudo", "gpg", "--dearmor", "--yes", "-o", str(_ROS_KEYRING), tmp_path])
    finally:
        os.unlink(tmp_path)
    write("[green][ok][/green] Signing key installed")
    _prog(65)

    arch = subprocess.check_output(["dpkg", "--print-architecture"], text=True).strip()
    codename = subprocess.check_output(
        ["bash", "-c", ". /etc/os-release && echo $UBUNTU_CODENAME"], text=True
    ).strip()
    sources_line = (
        f"deb [arch={arch} signed-by={_ROS_KEYRING}] "
        f"http://packages.ros.org/ros2/ubuntu {codename} main\n"
    )
    proc = subprocess.run(
        ["sudo", "tee", str(_ROS_SOURCES)],
        input=sources_line, capture_output=True, text=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"Failed to write {_ROS_SOURCES}")
    _prog(70)

    write("[cyan][*][/cyan] Updating apt cache...")
    _run_apt_with_progress(
        ["sudo", "apt-get", "update", "-q"] + _APT_TIMEOUTS,
        write,
        lambda p: _prog(70 + p * 0.30),
        _APT_ENV,
    )
    write("[green][ok][/green] ROS2 repository ready")
    _prog(100)


# Install the full ROS2 Jazzy Desktop and the developer tools (colcon, rosdep, etc.).
# Устанавливаем полный ROS2 Jazzy Desktop и инструменты разработчика (colcon, rosdep и т.д.).
def _install_ros2_jazzy(write: Write, on_progress: Optional[Callable[[float], None]] = None) -> None:
    write("[cyan][*][/cyan] Installing ros-jazzy-desktop and ros-dev-tools...")
    _run_apt_with_progress(
        ["sudo", "apt-get", "install", "-y", "ros-jazzy-desktop", "ros-dev-tools"] + _APT_TIMEOUTS,
        write,
        on_progress or (lambda _: None),
        _APT_ENV,
    )
    write("[green][ok][/green] ROS2 Jazzy Desktop installed")


# Install colcon if it is not already available. It is used to build the project packages.
# Устанавливаем colcon если он ещё не доступен. Он используется для сборки пакетов проекта.
def _install_colcon(write: Write) -> None:
    if shutil.which("colcon"):
        write("[green][ok][/green] colcon already available")
        return
    write("[cyan][*][/cyan] Installing colcon...")
    _run_quiet(
        ["sudo", "apt-get", "install", "-y", "--no-install-recommends",
         "python3-colcon-common-extensions"],
        write, _APT_ENV,
    )
    write("[green][ok][/green] colcon installed")


# Add "source /opt/ros/jazzy/setup.bash" to the user's shell config file.
# This makes ROS2 commands available in every new terminal session.
# Добавляем "source /opt/ros/jazzy/setup.bash" в конфиг оболочки пользователя.
# Это делает команды ROS2 доступными в каждой новой сессии терминала.
def _setup_shell_rc(write: Write) -> None:
    shell_name = Path(os.environ.get("SHELL", "/bin/bash")).name
    rc = Path.home() / (".zshrc" if shell_name == "zsh" else ".bashrc")
    source_line = "source /opt/ros/jazzy/setup.bash"
    if rc.exists() and source_line in rc.read_text():
        write(f"[green][ok][/green] ROS2 setup already in {rc.name}")
        return
    with rc.open("a") as f:
        f.write(f"\n# ROS2 Jazzy\n{source_line}\n")
    write(f"[green][ok][/green] Added ROS2 setup to ~/{rc.name}")


# Full ROS2 Jazzy installation split into 5 clearly visible steps with individual progress ranges.
# Полная установка ROS2 Jazzy, разбитая на 5 наглядных шагов с отдельными диапазонами прогресса.
def _task_install_jazzy(screen: LogScreen) -> None:
    try:
        # Step 1 — locale  (0 → 5 %)
        screen.set_progress(0, "Setting up locale...")
        screen.write("[bold]Step 1 / 5 — Locale[/bold]")
        _setup_locale(screen.write)

        # Step 2 — ROS2 repo  (5 → 20 %)
        screen.set_progress(5, "Adding ROS2 repository...")
        screen.write("\n[bold]Step 2 / 5 — ROS2 repository[/bold]")
        _add_ros2_repo(
            screen.write,
            on_progress=lambda p: screen.set_progress(5 + p * 0.15),
        )

        # Step 3 — ROS2 Jazzy  (20 → 85 %)
        screen.set_progress(20, "Installing ROS2 Jazzy Desktop...")
        screen.write("\n[bold]Step 3 / 5 — ROS2 Jazzy Desktop[/bold]")
        _install_ros2_jazzy(
            screen.write,
            on_progress=lambda p: screen.set_progress(20 + p * 0.65),
        )

        # Step 4 — colcon  (85 → 92 %)
        screen.set_progress(85, "Installing colcon...")
        screen.write("\n[bold]Step 4 / 5 — colcon[/bold]")
        _install_colcon(screen.write)

        # Step 5 — shell rc  (92 → 100 %)
        screen.set_progress(92, "Configuring shell...")
        screen.write("\n[bold]Step 5 / 5 — Shell configuration[/bold]")
        _setup_shell_rc(screen.write)
        screen.set_progress(100, "Done")

        screen.write(
            "\nRestart the terminal, then run [bold]cobot local-setup[/bold] again to build."
        )
        screen.finish(True)
    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)


# Build all project packages with colcon and track progress by counting finished packages.
# Собираем все пакеты проекта с помощью colcon и отслеживаем прогресс по количеству завершённых пакетов.
def _task_build(screen: LogScreen) -> None:
    try:
        if not shutil.which("colcon"):
            screen.write("[red]colcon not found.[/red]")
            screen.write("Source ROS2 first:  [bold]source /opt/ros/jazzy/setup.bash[/bold]")
            screen.finish(False)
            return

        # Count packages first so we can show X/total in the progress label.
        # Сначала считаем пакеты, чтобы показывать X/всего в подписи прогресса.
        list_result = subprocess.run(
            ["colcon", "list"], capture_output=True, text=True, cwd=_PROJECT_DIR,
        )
        total = max(len([l for l in list_result.stdout.splitlines() if l.strip()]), 1)

        screen.write(f"[bold]Building {total} package(s) with colcon[/bold]\n")
        screen.set_progress(0, f"0 / {total} packages done")
        built = 0

        def _track(line: str) -> None:
            nonlocal built
            screen.write(line)
            # colcon prints "Finished <<<" or "Failed <<<" when each package is done.
            # colcon печатает "Finished <<<" или "Failed <<<" когда каждый пакет готов.
            if "Finished <<<" in line or "Failed <<<" in line:
                built += 1
                screen.set_progress(built / total * 100, f"{built} / {total} packages done")

        _run_logged(["colcon", "build", "--symlink-install"], _track, cwd=_PROJECT_DIR)

        screen.set_progress(100, "Build complete")
        screen.write("\nActivate workspace:  [bold]source install/setup.bash[/bold]")
        screen.finish(True)
    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)


# Webots version that matches the Docker images used in this project.
# Версия Webots, соответствующая Docker-образам используемым в этом проекте.
_WEBOTS_VERSION = "2025a"
_WEBOTS_DEB_URL = (
    f"https://github.com/cyberbotics/webots/releases/download/"
    f"R{_WEBOTS_VERSION}/webots_{_WEBOTS_VERSION}_amd64.deb"
)


def webots_installed() -> bool:
    """Return True if Webots is available on PATH."""
    return shutil.which("webots") is not None


# Download the Webots .deb from GitHub and install it with apt.
# Progress: download (0-65%), apt install (65-100%).
# Скачиваем .deb Webots с GitHub и устанавливаем через apt.
# Прогресс: скачивание (0-65%), установка apt (65-100%).
def _task_install_webots(screen: LogScreen) -> None:
    try:
        screen.write(f"[bold]Installing Webots {_WEBOTS_VERSION}[/bold]\n")

        with tempfile.TemporaryDirectory() as tmp:
            deb_path = Path(tmp) / f"webots_{_WEBOTS_VERSION}_amd64.deb"

            screen.write(f"[dim]{_WEBOTS_DEB_URL}[/dim]\n")
            screen.set_progress(0, "Downloading Webots...")

            # urllib calls this hook periodically with how many bytes have been downloaded.
            # urllib вызывает этот обратный вызов периодически с количеством скачанных байт.
            def _hook(blocks: int, block_size: int, total: int) -> None:
                if total > 0:
                    pct = min(blocks * block_size / total * 65, 65)
                    mb = blocks * block_size / 1_048_576
                    total_mb = total / 1_048_576
                    screen.set_progress(pct, f"Downloading... {mb:.0f} / {total_mb:.0f} MB")

            urllib.request.urlretrieve(_WEBOTS_DEB_URL, deb_path, _hook)
            screen.write("[green]Download complete.[/green]")
            screen.set_progress(65, "Installing package...")

            proc = subprocess.Popen(
                ["sudo", "apt-get", "install", "-y", str(deb_path)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True,
            )
            for line in proc.stdout:
                s = line.rstrip()
                if s:
                    screen.write(s)
            proc.wait()

        if proc.returncode != 0:
            screen.write("\n[red]Installation failed.[/red]")
            screen.finish(False)
            return

        screen.set_progress(100, "Done")
        screen.write("\n[green]Webots installed successfully.[/green]")
        screen.finish(True)
    except Exception as exc:
        screen.write(f"\n[red]Error:[/red] {exc}")
        screen.finish(False)


# Minimal single-question app used between steps where a full wizard is not needed.
# Минимальное приложение с одним вопросом, используемое между шагами где полный мастер не нужен.
class _Ask(App[Optional[str]]):
    CSS = SCREEN_CSS

    def __init__(self, step: str, question: str, options: list, default: str):
        super().__init__()
        self._step = step
        self._question = question
        self._options = options
        self._default = default

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(self._step, self._question, self._options, self._default),
            self.exit,
        )


def _ask(step: str, question: str, options: list, default: str) -> Optional[str]:
    return _Ask(step, question, options, default).run()


# Public app used by run.py to install Webots before launching locally.
# Публичное приложение, используемое run.py для установки Webots перед локальным запуском.
class WebotsInstallApp(App[bool]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            LogScreen(f"Installing Webots {_WEBOTS_VERSION}", _task_install_webots, show_progress=True),
            self.exit,
        )



# Ask the user if they want to install ROS2 Jazzy, then run the installer if they say yes.
# Спрашиваем пользователя хочет ли он установить ROS2 Jazzy, и запускаем установщик если да.
class _InstallJazzyApp(App[None]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(
                "ROS2 not found",
                "ROS2 Jazzy is not installed. Install it now?",
                ["Yes, install ROS2 Jazzy", "No, skip"],
                "Yes, install ROS2 Jazzy",
            ),
            self._on_choice,
        )

    def _on_choice(self, choice: Optional[str]) -> None:
        if choice is None or choice.startswith("No"):
            self.exit()
            return
        self.push_screen(
            LogScreen("Installing ROS2 Jazzy", _task_install_jazzy, show_progress=True),
            lambda _: self.exit(),
        )


# Run the colcon build without asking any questions - used when ROS2 is already installed.
# Запускаем сборку colcon без лишних вопросов - используется когда ROS2 уже установлен.
class _BuildApp(App[None]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            LogScreen("Building project", _task_build, show_progress=True),
            lambda _: self.exit(),
        )


# Shown when the OS is not Ubuntu 24.04. Offers to fall back to docker-setup instead.
# Показывается когда ОС не Ubuntu 24.04. Предлагает перейти к docker-setup вместо этого.
class _DockerPromptApp(App[bool]):
    CSS = SCREEN_CSS

    def on_mount(self) -> None:
        self.push_screen(
            PickScreen(
                "Unsupported OS",
                "Ubuntu 24.04 not detected. Build a Docker image for development?",
                ["Yes, run docker-setup", "No, exit"],
                "Yes, run docker-setup",
            ),
            lambda v: self.exit(v is not None and v.startswith("Yes")),
        )



def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser(
        "local-setup",
        help="Install ROS2 Jazzy natively and build the project with colcon",
    )
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    # If this is not Ubuntu 24.04 we cannot install ROS2 Jazzy natively - offer Docker instead.
    # Если это не Ubuntu 24.04 мы не можем установить ROS2 Jazzy нативно - предлагаем Docker вместо этого.
    if not _detect_ubuntu_2404():
        if _DockerPromptApp().run():
            _docker_setup(args)
        return

    # ROS2 not installed yet - show the installer.
    # After installation the user must restart the terminal, so we stop here.
    # ROS2 ещё не установлен - показываем установщик.
    # После установки пользователь должен перезапустить терминал, поэтому останавливаемся здесь.
    if not _detect_ros2_jazzy():
        # Cache the sudo token now, while the terminal is in normal mode and the password
        # prompt is visible. Once Textual takes over the screen, sudo prompts become invisible
        # and the process hangs silently waiting for input that never arrives.
        # Кешируем sudo-токен сейчас, пока терминал работает в обычном режиме и запрос пароля
        # виден пользователю. После запуска Textual sudo не может показать запрос и процесс
        # зависает молча, ожидая ввод который никогда не придёт.
        subprocess.run(["sudo", "-v"], check=False)
        _InstallJazzyApp().run()
        return

    # ROS2 is ready - build the project.
    # ROS2 готов - собираем проект.
    _BuildApp().run()

    # Ask about Webots only after a successful build, and only if it is not already installed.
    # Спрашиваем про Webots только после успешной сборки и только если он ещё не установлен.
    if not webots_installed():
        v = _ask(
            "Optional: Webots",
            f"Install Webots {_WEBOTS_VERSION} simulator? (can also be done later via cobot run)",
            [f"Yes, install Webots {_WEBOTS_VERSION}", "No, skip"],
            "No, skip",
        )
        if v and v.startswith("Yes"):
            # Same sudo pre-cache before launching the Webots installer TUI.
            # Тот же предварительный кеш sudo перед запуском TUI установщика Webots.
            subprocess.run(["sudo", "-v"], check=False)
            WebotsInstallApp().run()
