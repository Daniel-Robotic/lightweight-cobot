from __future__ import annotations

import atexit
import os
import signal
import subprocess
import threading
from typing import Callable, Dict, List, Optional, Sequence

from rich.progress import BarColumn, Progress, SpinnerColumn, TextColumn

from cobot.ui import console, done, header

# Type of an optional callback invoked for every streamed output line.
# Тип опционального колбэка, вызываемого для каждой строки потокового вывода.
LineHook = Callable[[str], None]

# Registry of all live subprocesses, so the signal handler can kill them on exit.
# Maps pid -> Popen. Guarded by a lock because procs start/finish in helper calls.
# Реестр всех живых подпроцессов, чтобы обработчик сигнала мог убить их при выходе.
# Сопоставляет pid -> Popen. Защищён блокировкой, т.к. процессы создаются/завершаются в хелперах.
_procs: Dict[int, subprocess.Popen] = {}
_procs_lock = threading.Lock()
_handlers_installed = False


def _register(proc: subprocess.Popen) -> None:
    with _procs_lock:
        _procs[proc.pid] = proc


def _unregister(proc: subprocess.Popen) -> None:
    with _procs_lock:
        _procs.pop(proc.pid, None)


def _kill_proc(proc: subprocess.Popen) -> None:
    """Terminate a process and everything it spawned.

    Three strategies, in order of how the process was started:
      * a custom kill_fn (e.g. ``docker kill <container>``) registered on the proc;
      * a new-session process (e.g. ros2 launch) — every node shares the session, so
        ``pkill -s <sid>`` reaches all of them (killpg would only hit the launcher);
      * otherwise the process group (SIGTERM then SIGKILL), or the bare process.

    Завершает процесс и всё, что он породил. Три стратегии по способу запуска:
    пользовательский kill_fn (например ``docker kill``); процесс в новой сессии
    (ros2 launch — все узлы делят сессию, поэтому ``pkill -s`` достаёт каждый);
    иначе группа процессов (SIGTERM→SIGKILL) или сам процесс.
    """
    if proc.poll() is not None:
        return

    kill_fn = getattr(proc, "_cobot_kill_fn", None)
    if kill_fn is not None:
        try:
            kill_fn()
            try:
                proc.wait(timeout=5)
                return
            except subprocess.TimeoutExpired:
                pass
        except Exception:
            pass

    if getattr(proc, "_cobot_new_session", False):
        try:
            sid = os.getsid(proc.pid)
            subprocess.run(["pkill", "-TERM", "-s", str(sid)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                proc.wait(timeout=3)
                return
            except subprocess.TimeoutExpired:
                subprocess.run(["pkill", "-KILL", "-s", str(sid)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return
        except Exception:
            pass

    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass


def kill_all() -> None:
    """Kill every registered subprocess. Used by the signal handler and atexit.
    Убивает каждый зарегистрированный подпроцесс. Используется обработчиком сигнала и atexit.
    """
    with _procs_lock:
        procs = list(_procs.values())
    for proc in procs:
        _kill_proc(proc)


def _on_sigint(signum, frame):  # noqa: ANN001
    """SIGINT handler: stop all children, print a cancel note, and exit non-zero.
    Обработчик SIGINT: останавливает всех потомков, печатает заметку об отмене и выходит с ненулём.
    """
    kill_all()
    console.print("\n[yellow]Прервано пользователем (Ctrl-C).[/yellow]")
    raise SystemExit(130)


def install_signal_handlers() -> None:
    """Install the SIGINT handler and atexit cleanup exactly once.
    Устанавливает обработчик SIGINT и очистку atexit ровно один раз.
    """
    global _handlers_installed
    if _handlers_installed:
        return
    _handlers_installed = True
    signal.signal(signal.SIGINT, _on_sigint)
    atexit.register(kill_all)


def spawn(
    cmd: Sequence[str],
    *,
    env: Optional[dict] = None,
    cwd: Optional[str] = None,
    new_session: bool = False,
    shell: bool = False,
    kill_fn: Optional[Callable] = None,
) -> subprocess.Popen:
    """Start a subprocess with merged stdout/stderr as text, register it, and return it.

    new_session=True puts the process in its own session/process-group so the whole
    tree (e.g. all ros2 launch nodes) can be torn down with one signal. kill_fn is an
    optional custom teardown (e.g. ``docker kill``) used by the cleanup logic.

    Запускает подпроцесс с объединённым stdout/stderr в текстовом режиме, регистрирует
    его и возвращает. new_session=True помещает процесс в собственную сессию/группу.
    kill_fn — опциональная функция завершения (например ``docker kill``) для очистки.
    """
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        cwd=cwd,
        start_new_session=new_session,
        shell=shell,
    )
    proc._cobot_new_session = new_session  # type: ignore[attr-defined]
    proc._cobot_kill_fn = kill_fn  # type: ignore[attr-defined]
    _register(proc)
    return proc


def stream(
    cmd: Sequence[str],
    *,
    env: Optional[dict] = None,
    cwd: Optional[str] = None,
    on_line: Optional[LineHook] = None,
    new_session: bool = False,
    shell: bool = False,
    echo: bool = True,
    kill_fn: Optional[Callable] = None,
) -> int:
    """Run a command and stream every output line to the console (and on_line hook).

    Returns the process exit code. SIGKILL (-9) / SIGTERM (-15) are returned as-is so
    callers can treat user cancellation differently from real failures.

    Запускает команду и транслирует каждую строку вывода в консоль (и в колбэк on_line).
    Возвращает код возврата процесса. SIGKILL (-9) / SIGTERM (-15) возвращаются как есть,
    чтобы вызывающий код мог отличать отмену пользователем от реальных ошибок.
    """
    proc = spawn(cmd, env=env, cwd=cwd, new_session=new_session, shell=shell, kill_fn=kill_fn)
    try:
        for line in proc.stdout:
            s = line.rstrip()
            if on_line is not None:
                on_line(s)
            elif echo and s:
                console.print(f"  [dim]{_escape(s)}[/dim]")
        proc.wait()
    finally:
        _unregister(proc)
    return proc.returncode


def _escape(s: str) -> str:
    """Escape Rich markup so raw command output is never interpreted as markup.
    Экранирует разметку Rich, чтобы сырой вывод команды не интерпретировался как разметка.
    """
    return s.replace("[", "\\[")


# A progress bar that sticks to the bottom while log lines scroll above it.
# Прогресс-бар, "прилипающий" к низу, пока строки лога прокручиваются над ним.
def make_progress() -> Progress:
    """Create a Progress with a spinner, bar, percentage, and description column.
    Создаёт Progress со спиннером, баром, процентами и колонкой описания.
    """
    return Progress(
        SpinnerColumn(),
        BarColumn(bar_width=30),
        TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
        TextColumn("[dim]{task.description}[/dim]"),
        console=console,
        transient=True,
    )


def run_step(
    title: str,
    cmd: Sequence[str],
    *,
    env: Optional[dict] = None,
    cwd: Optional[str] = None,
    new_session: bool = False,
    shell: bool = False,
    show_progress: bool = True,
    total: float = 100.0,
    parse_progress: Optional[Callable[[str], Optional[tuple]]] = None,
    on_line: Optional[LineHook] = None,
    success_msg: str = "",
    fail_msg: str = "",
    finish: bool = True,
) -> int:
    """Run one command as a self-contained "block": header, live log + progress, status.

    parse_progress(line) may return (pct, label) to advance the bar, or None to ignore.
    Returns the exit code. Prints a ✓/✗ line unless finish=False (used when chaining
    several commands under one header).

    Запускает одну команду как самодостаточный "блок": заголовок, живой лог + прогресс,
    статус. parse_progress(line) может вернуть (pct, label) для продвижения бара или None.
    Возвращает код возврата. Печатает строку ✓/✗, если finish=True (иначе — при цепочке
    нескольких команд под одним заголовком).
    """
    if title:
        header(title)

    if not show_progress:
        rc = stream(cmd, env=env, cwd=cwd, on_line=on_line,
                    new_session=new_session, shell=shell)
    else:
        progress = make_progress()
        with progress:
            task = progress.add_task("", total=total)

            def _line(s: str) -> None:
                if parse_progress is not None:
                    parsed = parse_progress(s)
                    if parsed is not None:
                        pct, label = parsed
                        progress.update(task, completed=pct,
                                        description=label or "")
                if on_line is not None:
                    on_line(s)
                elif s:
                    progress.console.print(f"  [dim]{_escape(s)}[/dim]")

            rc = stream(cmd, env=env, cwd=cwd, on_line=_line,
                        new_session=new_session, shell=shell)
            progress.update(task, completed=total)

    ok = rc in (0, -9, -15)
    if finish:
        if ok:
            done(True, success_msg or "Готово")
        else:
            done(False, fail_msg or f"Команда завершилась с кодом {rc}")
    return rc


# A live progress context for tasks that run several commands or Python work and
# need to drive the bar manually. Yields a small controller with .log()/.set().
# Живой контекст прогресса для задач, выполняющих несколько команд или Python-работу
# и управляющих баром вручную. Отдаёт небольшой контроллер с .log()/.set().
class StepProgress:
    """Manual progress controller used as a context manager.

    Usage:
        with StepProgress("Building") as p:
            p.set(10, "step one")
            p.log("some output")

    Ручной контроллер прогресса, используемый как менеджер контекста.
    """

    def __init__(self, title: str, total: float = 100.0, show: bool = True):
        if title:
            header(title)
        self._total = total
        self._show = show
        self._progress: Optional[Progress] = None
        self._task = None

    def __enter__(self) -> "StepProgress":
        if self._show:
            self._progress = make_progress()
            self._progress.__enter__()
            self._task = self._progress.add_task("", total=self._total)
        return self

    def set(self, pct: float, label: str = "") -> None:
        if self._progress is not None:
            self._progress.update(self._task, completed=pct, description=label or "")

    def log(self, line: str, style: str = "dim") -> None:
        out = self._progress.console if self._progress is not None else console
        if line == "":
            out.print()
        else:
            out.print(f"  [{style}]{_escape(line)}[/{style}]" if style else f"  {line}")

    def raw(self, renderable) -> None:
        """Print a pre-built Rich renderable/markup string without escaping.
        Печатает готовый Rich-объект/строку с разметкой без экранирования.
        """
        out = self._progress.console if self._progress is not None else console
        out.print(renderable)

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._progress is not None:
            self._progress.__exit__(exc_type, exc, tb)
            self._progress = None
