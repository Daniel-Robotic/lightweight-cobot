from __future__ import annotations

from typing import Callable, List, Optional

from textual import on
from textual.app import ComposeResult
from textual.binding import Binding
from textual.screen import Screen
from textual.widgets import Footer, Input, LoadingIndicator, ProgressBar, RadioButton, RadioSet, RichLog, Static

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


class PickScreen(Screen[Optional[str]]):
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
        self.app.exit(None)


class InputScreen(Screen[Optional[str]]):
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


class LogScreen(Screen[bool]):
    """Streams task output into a scrollable log; press Enter to close when done."""

    BINDINGS = [Binding("enter,escape", "close", "Close", show=False)]

    def __init__(self, title: str, task: Callable[[LogScreen], None], show_progress: bool = False):
        super().__init__()
        self._title = title
        self._run_fn = task
        self._finished = False
        self._success = False
        self._show_progress = show_progress

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
        self.app.run_worker(lambda: self._run_fn(self), thread=True)

    def set_progress(self, pct: float, label: str = "") -> None:
        """Thread-safe: update the progress bar and optional step label."""
        if self._show_progress:
            self.app.call_from_thread(self._do_set_progress, pct, label)

    def _do_set_progress(self, pct: float, label: str) -> None:
        self.query_one("#progress", ProgressBar).progress = pct
        if label:
            self.query_one("#step-label", Static).update(label)

    def write(self, line: str) -> None:
        """Thread-safe: append a line to the log."""
        self.app.call_from_thread(self._append, line)

    def _append(self, line: str) -> None:
        self.query_one(RichLog).write(line)

    def finish(self, success: bool) -> None:
        """Thread-safe: mark task done and prompt the user to close."""
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
        if self._finished:
            self.dismiss(self._success)


class RunScreen(Screen[None]):
    """Streams a long-running process. S/Enter/Escape stops or closes."""

    BINDINGS = [
        Binding("s", "stop_close", "Stop", show=True, priority=True),
        Binding("enter", "stop_close", "Close", show=False),
        Binding("escape", "stop_close", "Close", show=False),
    ]

    def __init__(self, title: str, task: Callable[[RunScreen], None]):
        super().__init__()
        self._title = title
        self._run_fn = task
        self._proc = None          # set via set_proc()
        self._kill_fn = None       # optional custom kill callable
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
        self.app.run_worker(lambda: self._run_fn(self), thread=True)

    def set_proc(self, proc) -> None:
        """Register the running subprocess so Stop can terminate it."""
        self._proc = proc

    def set_kill_fn(self, fn: Callable) -> None:
        """Override the default terminate() with a custom kill function."""
        self._kill_fn = fn

    def write(self, line: str) -> None:
        self.app.call_from_thread(self._append, line)

    def _append(self, line: str) -> None:
        self.query_one(RichLog).write(line)

    def finish(self, stopped: bool = False) -> None:
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
        self.write("\n[yellow]Stopping process...[/yellow]")
