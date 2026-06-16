from __future__ import annotations

import os
import select as _select
import sys
from typing import List, Optional, Sequence, Tuple

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.text import Text

# Raw terminal control is POSIX-only; the project targets Linux/ROS so this is fine.
# Сырой режим терминала только для POSIX; проект под Linux/ROS, так что всё в порядке.
try:
    import termios
    import tty
    _HAS_TERMIOS = True
except ImportError:  # pragma: no cover - Windows fallback
    _HAS_TERMIOS = False

# Single shared console used everywhere so styling and width stay consistent.
# Единый общий console, используемый везде, чтобы стиль и ширина были согласованы.
console = Console(highlight=False)

# Glyphs used across the UI. Kept here so the whole look can be retuned in one place.
# Глифы, используемые в интерфейсе. Собраны здесь, чтобы весь вид настраивался в одном месте.
_CURSOR = "❯"
_OK = "✓"
_FAIL = "✗"
_CHECK_ON = "◼"
_CHECK_OFF = "◻"


def is_interactive() -> bool:
    """Return True if both stdin and stdout are real terminals.

    Arrow-key selection needs a real TTY to read raw key presses. When that is
    not available (piped input, CI) callers should fall back to defaults.

    Возвращает True, если и stdin, и stdout являются настоящими терминалами.
    Выбор стрелками требует реального TTY для чтения нажатий клавиш. Если его нет
    (перенаправленный ввод, CI), вызывающий код должен использовать значения по умолчанию.
    """
    try:
        return sys.stdin.isatty() and sys.stdout.isatty()
    except Exception:
        return False


# Low-level key reader
# Низкоуровневое чтение клавиш
# How long to wait (seconds) after a lone ESC byte before deciding it is really the
# Escape key and not the start of an arrow escape sequence (\x1b[A etc.).
# Сколько ждать (секунд) после одиночного байта ESC, прежде чем решить, что это
# именно клавиша Escape, а не начало escape-последовательности стрелок (\x1b[A и т.п.).
_ESC_TIMEOUT = 0.05


def _read_key() -> Tuple[str, str]:
    """Read one key press in raw mode and classify it.

    Returns a (kind, char) tuple where kind is one of: "up", "down", "enter", "esc",
    "space", "backspace", "char", "interrupt", "other". This is used instead of
    readchar because readchar blocks after a lone ESC (waiting to see whether it is an
    arrow sequence); here a short select() timeout distinguishes a real Escape press.
    UTF-8 multibyte input (e.g. Cyrillic in a password) is decoded fully.

    Читает одно нажатие в сыром режиме и классифицирует его. Возвращает кортеж
    (kind, char). Используется вместо readchar, потому что readchar зависает после
    одиночного ESC (ожидая, не последовательность ли это стрелок); здесь короткий
    таймаут select() отличает настоящий Escape. UTF-8 (например кириллица в пароле)
    декодируется полностью.
    """
    if not _HAS_TERMIOS:  # pragma: no cover
        ch = sys.stdin.read(1)
        return ("char", ch)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        b = os.read(fd, 1)
        if not b:
            return ("other", "")
        c = b[0]

        if c == 0x1B:  # ESC — could be a lone Escape or an arrow/escape sequence
            ready, _, _ = _select.select([fd], [], [], _ESC_TIMEOUT)
            if not ready:
                return ("esc", "")
            seq = os.read(fd, 3)
            last = seq[-1:] if seq else b""
            if last == b"A":
                return ("up", "")
            if last == b"B":
                return ("down", "")
            if last in (b"C", b"D"):
                return ("other", "")
            return ("esc", "")
        if c in (0x0D, 0x0A):       # Enter
            return ("enter", "")
        if c == 0x03:               # Ctrl-C (raw mode swallows SIGINT)
            return ("interrupt", "")
        if c == 0x20:               # Space
            return ("space", " ")
        if c in (0x7F, 0x08):       # Backspace / Delete
            return ("backspace", "")
        if c < 0x20:                # other control char — ignore
            return ("other", "")

        # Printable byte — read any UTF-8 continuation bytes so multibyte chars decode.
        # Печатный байт — дочитываем продолжения UTF-8, чтобы многобайтовые символы декодировались.
        extra = 0
        if c >= 0xF0:
            extra = 3
        elif c >= 0xE0:
            extra = 2
        elif c >= 0xC0:
            extra = 1
        if extra:
            b += os.read(fd, extra)
        return ("char", b.decode("utf-8", errors="ignore"))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# Block headers and footers
# Заголовки и завершения блоков
def header(title: str, subtitle: str = "") -> None:
    """Print a styled header block that marks the start of a task or wizard step.
    Печатает стилизованный блок-заголовок, обозначающий начало задачи или шага мастера.
    """
    console.print()
    bar = Text("▌ ", style="bold cyan")
    bar.append(title, style="bold")
    if subtitle:
        bar.append(f"  {subtitle}", style="dim")
    console.print(bar)


def done(success: bool, message: str = "") -> None:
    """Print the final status line of a task (green ✓ on success, red ✗ on failure).
    Печатает финальную строку статуса задачи (зелёная ✓ при успехе, красная ✗ при ошибке).
    """
    if success:
        line = Text(f"{_OK} ", style="bold green")
        line.append(message or "Done", style="green")
    else:
        line = Text(f"{_FAIL} ", style="bold red")
        line.append(message or "Failed", style="red")
    console.print(line)


def note(message: str) -> None:
    """Print a dim helper/info line.
    Печатает приглушённую вспомогательную/информационную строку.
    """
    console.print(Text(f"  {message}", style="dim"))


def info(message: str) -> None:
    """Print a plain message through the shared console (Rich markup allowed).
    Печатает обычное сообщение через общий console (разрешена разметка Rich).
    """
    console.print(message)


def error(message: str) -> None:
    """Print an error line.
    Печатает строку ошибки.
    """
    console.print(f"[bold red]Error:[/bold red] {message}")


# A collapsed answer line, printed after an interactive block is resolved.
# Свёрнутая строка-ответ, печатается после разрешения интерактивного блока.
def _print_answer(question: str, answer: str) -> None:
    line = Text(f"{_OK} ", style="bold green")
    line.append(f"{question} ", style="dim")
    line.append("· ", style="dim")
    line.append(answer, style="bold")
    console.print(line)


def _print_cancelled(question: str) -> None:
    line = Text(f"{_FAIL} ", style="bold red")
    line.append(f"{question} ", style="dim")
    line.append("· cancelled", style="red")
    console.print(line)


def _render_choices(question: str, options: Sequence[str], cursor: int,
                    note_text: str = "") -> Panel:
    """Build the renderable shown while the user is navigating a single-choice list.
    Строит отрисовываемый объект, показываемый пока пользователь навигирует по списку выбора.
    """
    rows: List[Text] = []
    for i, opt in enumerate(options):
        if i == cursor:
            row = Text(f" {_CURSOR} ", style="bold cyan")
            row.append(opt, style="bold")
        else:
            row = Text(f"   {opt}", style="dim")
        rows.append(row)
    body = Group(*rows)
    title = Text(question, style="bold")
    sub = "↑/↓ — выбор · Enter — подтвердить · Esc — отмена"
    if note_text:
        sub = f"{note_text}\n{sub}"
    return Panel(body, title=title, title_align="left", subtitle=Text(sub, style="dim"),
                 subtitle_align="left", border_style="cyan", padding=(0, 1))


def select(question: str, options: Sequence[str], default: Optional[str] = None,
           note: str = "") -> Optional[str]:
    """Show an arrow-key single-choice block and return the chosen option string.

    Returns None if the user pressed Escape / Ctrl-C. When the terminal is not
    interactive the default (or first option) is returned without prompting.

    Показывает блок выбора одного варианта со стрелками и возвращает выбранную строку.
    Возвращает None, если пользователь нажал Escape / Ctrl-C. Если терминал не
    интерактивный, возвращается значение по умолчанию (или первый вариант) без запроса.
    """
    options = list(options)
    if not options:
        return None
    cursor = options.index(default) if default in options else 0

    if not is_interactive():
        chosen = options[cursor]
        _print_answer(question, chosen)
        return chosen

    with Live(_render_choices(question, options, cursor, note), console=console,
              auto_refresh=False, transient=True) as live:
        while True:
            live.update(_render_choices(question, options, cursor, note), refresh=True)
            kind, ch = _read_key()
            if kind == "up" or (kind == "char" and ch == "k"):
                cursor = (cursor - 1) % len(options)
            elif kind == "down" or (kind == "char" and ch == "j"):
                cursor = (cursor + 1) % len(options)
            elif kind == "enter":
                break
            elif kind in ("esc", "interrupt"):
                _print_cancelled(question)
                return None

    chosen = options[cursor]
    _print_answer(question, chosen)
    return chosen


def _render_multi(question: str, options: Sequence[str], cursor: int,
                  selected: set, note_text: str = "") -> Panel:
    """Build the renderable for a multi-choice checkbox list.
    Строит отрисовываемый объект для списка множественного выбора с чекбоксами.
    """
    rows: List[Text] = []
    for i, opt in enumerate(options):
        box = _CHECK_ON if i in selected else _CHECK_OFF
        if i == cursor:
            row = Text(f" {_CURSOR} {box} ", style="bold cyan")
            row.append(opt, style="bold")
        else:
            row = Text(f"   {box} ", style="green" if i in selected else "dim")
            row.append(opt, style="" if i in selected else "dim")
        rows.append(row)
    body = Group(*rows)
    title = Text(question, style="bold")
    sub = "↑/↓ — навигация · Space — отметить · Enter — подтвердить · Esc — отмена"
    if note_text:
        sub = f"{note_text}\n{sub}"
    return Panel(body, title=title, title_align="left", subtitle=Text(sub, style="dim"),
                 subtitle_align="left", border_style="cyan", padding=(0, 1))


def multiselect(question: str, options: Sequence[str],
                defaults: Optional[Sequence[str]] = None,
                note: str = "") -> Optional[List[str]]:
    """Show an arrow-key multi-choice block. Space toggles, Enter confirms.

    Returns the list of selected option strings, or None if cancelled.

    Показывает блок множественного выбора со стрелками. Space переключает, Enter подтверждает.
    Возвращает список выбранных строк или None при отмене.
    """
    options = list(options)
    if not options:
        return []
    if defaults is None:
        selected = set(range(len(options)))
    else:
        selected = {i for i, o in enumerate(options) if o in defaults}
    cursor = 0

    if not is_interactive():
        chosen = [options[i] for i in sorted(selected)]
        _print_answer(question, ", ".join(chosen) or "—")
        return chosen

    with Live(_render_multi(question, options, cursor, selected, note), console=console,
              auto_refresh=False, transient=True) as live:
        while True:
            live.update(_render_multi(question, options, cursor, selected, note), refresh=True)
            kind, ch = _read_key()
            if kind == "up" or (kind == "char" and ch == "k"):
                cursor = (cursor - 1) % len(options)
            elif kind == "down" or (kind == "char" and ch == "j"):
                cursor = (cursor + 1) % len(options)
            elif kind == "space":
                selected.symmetric_difference_update({cursor})
            elif kind == "enter":
                break
            elif kind in ("esc", "interrupt"):
                _print_cancelled(question)
                return None

    chosen = [options[i] for i in sorted(selected)]
    _print_answer(question, ", ".join(chosen) or "—")
    return chosen


def text(question: str, default: str = "", note: str = "") -> Optional[str]:
    """Prompt for a single line of free text, pre-filled with default.

    Returns the entered value (or default if left empty), or None on Ctrl-C / EOF.

    Запрашивает одну строку произвольного текста, предзаполненную значением по умолчанию.
    Возвращает введённое значение (или default, если пусто), либо None при Ctrl-C / EOF.
    """
    prompt = Text()
    prompt.append(f"{_CURSOR} ", style="bold cyan")
    prompt.append(question, style="bold")
    if default:
        prompt.append(f"  [{default}]", style="dim")
    console.print(prompt)
    if note:
        console.print(Text(f"  {note}", style="dim"))
    try:
        raw = input("  > ").strip()
    except (EOFError, KeyboardInterrupt):
        console.print()
        _print_cancelled(question)
        return None
    value = raw or default
    return value


def confirm(question: str, default: bool = True) -> bool:
    """Yes/No selection block. Returns True for yes, False for no or cancel.
    Блок выбора Да/Нет. Возвращает True для да, False для нет или отмены.
    """
    yes, no = "Да", "Нет"
    choice = select(question, [yes, no], default=yes if default else no)
    return choice == yes
