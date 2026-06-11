from __future__ import annotations

import subprocess
import sys
import threading
from typing import List, Optional, Sequence

from cobot import ui
from cobot.ui import console

# How often the keep-alive thread refreshes the sudo timestamp (seconds).
# Sudo's default timeout is 15 min; 60 s gives a huge safety margin.
# Как часто поток keep-alive обновляет токен sudo (секунды).
# Таймаут sudo по умолчанию 15 мин; 60 с даёт большой запас.
_KEEPALIVE_INTERVAL = 60

# Module-level state: whether sudo has been primed and the keep-alive thread.
# Состояние уровня модуля: прогрет ли sudo и поток keep-alive.
_primed = False
_keepalive_thread: Optional[threading.Thread] = None
_keepalive_stop = threading.Event()


def _have_valid_timestamp() -> bool:
    """Return True if a non-interactive ``sudo -n -v`` succeeds (cached token valid).
    Возвращает True, если ``sudo -n -v`` проходит без запроса (токен закеширован и валиден).
    """
    return subprocess.run(
        ["sudo", "-n", "-v"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0


def _read_masked_password(prompt: str) -> Optional[str]:
    """Read a password character-by-character, echoing a ``★`` for each one.

    Backspace deletes the last char. Enter submits. Ctrl-C / Esc cancels (None).
    Falls back to getpass when stdin is not a TTY.

    Читает пароль посимвольно, отображая ``★`` за каждый символ.
    Backspace удаляет последний символ. Enter — подтвердить. Ctrl-C / Esc — отмена (None).
    Откатывается на getpass, если stdin не является TTY.
    """
    if not sys.stdin.isatty():
        import getpass
        try:
            return getpass.getpass(prompt)
        except (EOFError, KeyboardInterrupt):
            return None

    sys.stdout.write(prompt)
    sys.stdout.flush()
    chars: List[str] = []
    while True:
        kind, ch = ui._read_key()
        if kind == "enter":
            sys.stdout.write("\n")
            sys.stdout.flush()
            return "".join(chars)
        if kind in ("esc", "interrupt"):
            sys.stdout.write("\n")
            sys.stdout.flush()
            return None
        if kind == "backspace":
            if chars:
                chars.pop()
                # Erase one mask glyph: move back, overwrite with space, move back.
                # Стираем один символ маски: назад, пробел, снова назад.
                sys.stdout.write("\b \b")
                sys.stdout.flush()
            continue
        # Space and any printable char are part of the password.
        # Пробел и любой печатный символ — часть пароля.
        if kind == "space":
            chars.append(" ")
            sys.stdout.write("★")
            sys.stdout.flush()
        elif kind == "char" and ch.isprintable():
            chars.append(ch)
            sys.stdout.write("★")
            sys.stdout.flush()


def _validate_password(password: str) -> bool:
    """Feed the password to ``sudo -S -v`` to validate it and cache the timestamp.
    Передаёт пароль в ``sudo -S -v`` для проверки и кеширования токена.
    """
    proc = subprocess.run(
        ["sudo", "-S", "-v"],
        input=password + "\n",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        text=True,
    )
    return proc.returncode == 0


def _keepalive_loop() -> None:
    """Refresh the sudo timestamp periodically until the process exits.
    Периодически обновляет токен sudo, пока процесс не завершится.
    """
    while not _keepalive_stop.wait(_KEEPALIVE_INTERVAL):
        subprocess.run(
            ["sudo", "-n", "-v"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )


def _start_keepalive() -> None:
    global _keepalive_thread
    if _keepalive_thread is not None and _keepalive_thread.is_alive():
        return
    _keepalive_stop.clear()
    _keepalive_thread = threading.Thread(target=_keepalive_loop, daemon=True)
    _keepalive_thread.start()


def ensure_sudo() -> bool:
    """Make sure we hold a valid sudo timestamp, asking for the password once.

    If a valid cached timestamp already exists (e.g. the user ran sudo recently),
    no password is asked. Otherwise the user is prompted up to 3 times with a masked
    input. On success a keep-alive thread is started. Returns True if sudo is ready.

    Гарантирует наличие валидного токена sudo, спрашивая пароль один раз.
    Если валидный токен уже есть (например, пользователь недавно вызывал sudo), пароль
    не спрашивается. Иначе пользователю предлагается до 3 попыток с маскированным вводом.
    При успехе запускается поток keep-alive. Возвращает True, если sudo готов.
    """
    global _primed
    if _primed and _have_valid_timestamp():
        return True

    if _have_valid_timestamp():
        _primed = True
        _start_keepalive()
        return True

    console.print(
        "\n[bold]Для установки/удаления системных пакетов нужны права root.[/bold]"
    )
    console.print(
        "[dim]Пароль спросим один раз и будем держать сессию sudo активной "
        "до конца операции.[/dim]"
    )

    for attempt in range(3):
        password = _read_masked_password("  [sudo] пароль: ")
        if password is None:
            console.print("[yellow]Отменено.[/yellow]")
            return False
        if _validate_password(password):
            del password
            _primed = True
            _start_keepalive()
            console.print("[green]✓ sudo активирован[/green]")
            return True
        remaining = 2 - attempt
        if remaining > 0:
            console.print(f"[red]Неверный пароль.[/red] Осталось попыток: {remaining}")
        else:
            console.print("[red]Неверный пароль. Превышено число попыток.[/red]")
    return False


def sudo(cmd: Sequence[str]) -> List[str]:
    """Prefix a command with ``sudo -n`` (non-interactive; token already cached).
    Префиксует команду ``sudo -n`` (неинтерактивно; токен уже закеширован).
    """
    return ["sudo", "-n", *cmd]


def stop_keepalive() -> None:
    """Stop the keep-alive thread. Safe to call even if it was never started.
    Останавливает поток keep-alive. Безопасно вызывать, даже если он не запускался.
    """
    _keepalive_stop.set()
