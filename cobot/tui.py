from __future__ import annotations

from typing import Callable, List, Optional

from textual import on
from textual.app import ComposeResult
from textual.binding import Binding
from textual.screen import Screen
from textual.widgets import Footer, Input, LoadingIndicator, ProgressBar, RadioButton, RadioSet, RichLog, Static

# Shared CSS applied to every screen in the app.
# Общий CSS, применяемый ко всем экранам приложения.
SCREEN_CSS = """
Screen {
    padding: 2 4;
}
#step {
    color: $text-muted;
    text-style: dim;
}
#question {
    text-style: bold;
    color: $accent;
    margin-top: 1;
    margin-bottom: 1;
}
#note {
    color: $text-muted;
    text-style: dim;
    margin-bottom: 1;
}
RadioSet {
    height: auto;
    border: none;
    padding: 0;
    margin-bottom: 1;
}
Input {
    margin-bottom: 1;
}
LogScreen #progress {
    margin-top: 1;
    height: 1;
}
LogScreen #step-label {
    color: $text-muted;
    text-style: dim;
    margin-bottom: 1;
}
LogScreen #log {
    height: 1fr;
    border: none;
    padding: 0 1;
    margin-top: 1;
}
LogScreen #loading {
    height: 1;
    margin-top: 1;
}
LogScreen #hint {
    margin-top: 1;
    color: $text;
}
RunScreen #log {
    height: 1fr;
    border: none;
    padding: 0 1;
    margin-top: 1;
}
RunScreen #loading {
    height: 1;
    margin-top: 1;
}
RunScreen #hint {
    margin-top: 1;
    color: $text-muted;
    text-style: dim;
}
"""


# A screen that shows a question and a list of radio button options.
# The user picks one and presses Enter - the chosen string is returned as the result.
# Экран с вопросом и списком вариантов в виде радио-кнопок.
# Пользователь выбирает один и нажимает Enter - выбранная строка возвращается как результат.
class PickScreen(Screen[Optional[str]]):
    """Single-choice radio button screen. Returns the selected option string, or None on Escape.
    Экран выбора одного варианта с радио-кнопками. Возвращает выбранную строку или None при Escape.
    """
    BINDINGS = [
        Binding("enter", "submit", "Confirm", priority=True),
        Binding("escape", "abort", "Cancel"),
    ]

    def __init__(self, step: str, question: str, options: List[str], default: str, note: str = ""):
        super().__init__()
        self._step = step
        self._question = question
        self._options = options
        self._default = default
        self._note = note

    def compose(self) -> ComposeResult:
        yield Static(self._step, id="step")
        yield Static(self._question, id="question")
        if self._note:
            yield Static(self._note, id="note")
        with RadioSet(id="choices"):
            for opt in self._options:
                # Pre-select the default option so the user can just press Enter to accept it.
                # Заранее выделяем вариант по умолчанию, чтобы пользователь мог просто нажать Enter.
                yield RadioButton(opt, value=(opt == self._default))
        yield Footer()

    def on_mount(self) -> None:
        self.query_one(RadioSet).focus()

    def action_submit(self) -> None:
        radio_set = self.query_one("#choices", RadioSet)
        buttons = list(radio_set.query(RadioButton))
        idx = getattr(radio_set, "_selected", None)
        if idx is not None and 0 <= idx < len(buttons):
            self.dismiss(str(buttons[idx].label))
        else:
            btn = radio_set.pressed_button
            self.dismiss(str(btn.label) if btn else self._default)

    def action_abort(self) -> None:
        # Exit the whole app, not just this screen, so the calling code knows the user cancelled.
        # Выходим из всего приложения, а не только из этого экрана, чтобы вызывающий код знал об отмене.
        self.app.exit(None)


# A screen that shows a question with a free-text input field.
# The user types a value, presses Enter, and the text is returned as the result.
# Экран с вопросом и полем для ввода произвольного текста.
# Пользователь вводит значение, нажимает Enter, и текст возвращается как результат.
class InputScreen(Screen[Optional[str]]):
    """Free-text input screen. Returns the trimmed value on Enter, or None on Escape.
    Экран свободного ввода текста. Возвращает обрезанное значение при Enter или None при Escape.
    """
    BINDINGS = [
        Binding("enter", "submit", "Confirm", priority=True),
        Binding("escape", "abort", "Cancel"),
    ]

    def __init__(self, step: str, question: str, default: str, note: str = ""):
        super().__init__()
        self._step = step
        self._question = question
        self._default = default
        self._note = note

    def compose(self) -> ComposeResult:
        yield Static(self._step, id="step")
        yield Static(self._question, id="question")
        if self._note:
            yield Static(self._note, id="note")
        yield Input(id="value", value=self._default)
        yield Footer()

    def on_mount(self) -> None:
        self.query_one(Input).focus()

    @on(Input.Submitted)
    def _submitted(self, event: Input.Submitted) -> None:
        self.dismiss(event.value.strip() or self._default)

    def action_submit(self) -> None:
        val = self.query_one(Input).value.strip()
        self.dismiss(val or self._default)

    def action_abort(self) -> None:
        self.app.exit(None)


# A screen that streams output from a background task into a scrollable log.
# Used for long-running operations like installs and builds.
# Press Enter or Escape to close once the task finishes.
# Экран, который транслирует вывод фоновой задачи в прокручиваемый лог.
# Используется для долгих операций, таких как установка и сборка.
# После завершения задачи закрывается по нажатию Enter или Escape.
class LogScreen(Screen[bool]):
    """Log screen for long-running background tasks. Shows a scrollable log and optional
    progress bar. Returns True on success, False on failure after the task finishes.
    Экран лога для долгих фоновых задач. Показывает прокручиваемый лог и опциональный
    прогресс-бар. Возвращает True при успехе, False при ошибке после завершения задачи.
    """
    BINDINGS = [Binding("enter,escape", "close", "Close", show=False)]

    def __init__(self, title: str, task: Callable[[LogScreen], None], show_progress: bool = False):
        super().__init__()
        self._title = title
        self._run_fn = task
        self._finished = False
        self._success = False
        self._show_progress = show_progress
        # Tracks the subprocess that is currently running so on_unmount can kill it.
        # Отслеживает текущий subprocess, чтобы on_unmount мог его завершить.
        self._active_proc = None
        self._stopped = False

    def compose(self) -> ComposeResult:
        yield Static(self._title, id="step")
        if self._show_progress:
            yield ProgressBar(id="progress", total=100, show_eta=False)
            yield Static("", id="step-label")
        yield RichLog(id="log", highlight=True, markup=True, wrap=True)
        yield LoadingIndicator(id="loading")
        yield Static("", id="hint")
        yield Footer()

    def on_mount(self) -> None:
        self.query_one(RichLog).focus()
        # Run the task in a worker thread so the UI stays responsive.
        # Запускаем задачу в отдельном потоке, чтобы интерфейс не зависал.
        self.app.run_worker(lambda: self._run_fn(self), thread=True)

    def set_proc(self, proc) -> None:
        # Register the subprocess that is currently running.
        # Called from the worker thread - GIL makes simple assignment safe here.
        # Регистрируем текущий subprocess.
        # Вызывается из рабочего потока - простое присваивание безопасно благодаря GIL.
        self._active_proc = proc

    def is_stopped(self) -> bool:
        # Return True if the user has closed the screen before the task finished.
        # Возвращает True если пользователь закрыл экран до завершения задачи.
        return self._stopped

    def on_unmount(self) -> None:
        # Kill the active subprocess when the screen closes so it does not keep running in the background.
        # Убиваем активный subprocess при закрытии экрана, чтобы он не продолжал работать в фоне.
        self._stopped = True
        proc = self._active_proc
        if proc is not None:
            try:
                proc.kill()
            except Exception:
                pass

    def set_progress(self, pct: float, label: str = "") -> None:
        # Thread-safe - this is called from the worker thread, not the UI thread.
        # Потокобезопасно - вызывается из рабочего потока, а не из потока интерфейса.
        if self._show_progress:
            self.app.call_from_thread(self._do_set_progress, pct, label)

    def _do_set_progress(self, pct: float, label: str) -> None:
        self.query_one("#progress", ProgressBar).progress = pct
        if label:
            self.query_one("#step-label", Static).update(label)

    def write(self, line: str) -> None:
        # Thread-safe - append a line to the log from a worker thread.
        # Потокобезопасно - добавляет строку в лог из рабочего потока.
        self.app.call_from_thread(self._append, line)

    def _append(self, line: str) -> None:
        self.query_one(RichLog).write(line)

    def finish(self, success: bool) -> None:
        # Thread-safe - called by the task when it is done to show the close hint.
        # Потокобезопасно - вызывается задачей по завершении, чтобы показать подсказку о закрытии.
        self.app.call_from_thread(self._do_finish, success)

    def _do_finish(self, success: bool) -> None:
        self._finished = True
        self._success = success
        self.query_one("#loading", LoadingIndicator).display = False
        msg = (
            "[green]Done![/green]  Press [bold]Enter[/bold] to close."
            if success
            else "[red]Failed.[/red]  Press [bold]Enter[/bold] to close."
        )
        self.query_one("#hint", Static).update(msg)

    def action_close(self) -> None:
        # Only allow closing after the task has finished, not while it is still running.
        # Разрешаем закрытие только после завершения задачи, а не во время её работы.
        if self._finished:
            self.dismiss(self._success)


# A screen for a long-running process that the user can stop at any time.
# Shows a live log and offers S / Enter / Escape to stop or close.
# Экран для долго работающего процесса, который пользователь может остановить в любой момент.
# Показывает живой лог и предлагает S / Enter / Escape для остановки или закрытия.
class RunScreen(Screen[None]):
    """Run screen for a persistent process (e.g. ROS2 launch). Shows a live log and allows
    the user to stop the process with S or close after it exits with Enter/Escape.
    Экран запуска для постоянно работающего процесса (например ros2 launch). Показывает живой
    лог и позволяет остановить процесс клавишей S или закрыть после завершения через Enter/Escape.
    """
    BINDINGS = [
        Binding("s", "stop_close", "Stop", show=True, priority=True),
        Binding("enter", "stop_close", "Close", show=False),
        Binding("escape", "stop_close", "Close", show=False),
    ]

    def __init__(self, title: str, task: Callable[[RunScreen], None]):
        super().__init__()
        self._title = title
        self._run_fn = task
        self._proc = None          # the subprocess, set via set_proc()
        self._kill_fn = None       # optional custom kill callable, set via set_kill_fn()
        self._finished = False
        self._stopped = False

    def compose(self) -> ComposeResult:
        yield Static(self._title, id="step")
        yield RichLog(id="log", highlight=True, markup=True, wrap=True)
        yield LoadingIndicator(id="loading")
        yield Static("  Press [bold]S[/bold] to stop the process", id="hint")
        yield Footer()

    def on_mount(self) -> None:
        self.query_one(RichLog).focus()
        # Run the process task in a worker thread so the UI stays responsive.
        # Запускаем задачу с процессом в отдельном потоке, чтобы интерфейс не зависал.
        self.app.run_worker(lambda: self._run_fn(self), thread=True)

    def set_proc(self, proc) -> None:
        # Register the subprocess so the Stop button knows what to terminate.
        # Регистрируем subprocess, чтобы кнопка Stop знала что завершать.
        self._proc = proc

    def set_kill_fn(self, fn: Callable) -> None:
        # Override the default proc.terminate() with a custom kill function.
        # For example, docker kill or os.killpg for process groups.
        # Заменяем стандартный proc.terminate() кастомной функцией завершения.
        # Например, docker kill или os.killpg для групп процессов.
        self._kill_fn = fn

    def write(self, line: str) -> None:
        # Thread-safe - called from the worker thread to append a log line.
        # Потокобезопасно - вызывается из рабочего потока для добавления строки в лог.
        self.app.call_from_thread(self._append, line)

    def _append(self, line: str) -> None:
        self.query_one(RichLog).write(line)

    def finish(self, stopped: bool = False) -> None:
        # Thread-safe - called by the task when the process exits naturally.
        # Потокобезопасно - вызывается задачей когда процесс завершается естественным образом.
        self.app.call_from_thread(self._do_finish, stopped)

    def _do_finish(self, stopped: bool) -> None:
        self._finished = True
        self.query_one("#loading", LoadingIndicator).display = False
        if stopped:
            msg = "[yellow]Process stopped.[/yellow]  Press [bold]Enter[/bold] to close."
        else:
            msg = "[green]Process exited.[/green]  Press [bold]Enter[/bold] to close."
        self.query_one("#hint", Static).update(msg)

    def action_stop_close(self) -> None:
        # This runs in the UI thread, so we call _append() directly instead of write()
        # because write() uses call_from_thread() which only works from other threads.
        # Выполняется в потоке UI, поэтому вызываем _append() напрямую, а не write(),
        # потому что write() использует call_from_thread(), который работает только из других потоков.
        if self._finished:
            self.dismiss(None)
            return
        self._stopped = True
        if self._kill_fn is not None:
            try:
                self._kill_fn()
            except Exception:
                pass
        elif self._proc is not None and self._proc.poll() is None:
            try:
                self._proc.terminate()
            except Exception:
                pass
        self._append("\n[yellow]Stopping process...[/yellow]")
