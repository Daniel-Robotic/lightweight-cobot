from __future__ import annotations

from typing import Callable, List, Optional

from textual import on
from textual.app import ComposeResult
from textual.binding import Binding
from textual.screen import Screen
from textual.widgets import Footer, Input, RadioButton, RadioSet, RichLog, Static

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
LogScreen #log {
    height: 1fr;
    border: none;
    padding: 0 1;
    margin-top: 1;
}
LogScreen #hint {
    margin-top: 1;
    color: $text;
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

    def __init__(self, title: str, task: Callable[[LogScreen], None]):
        super().__init__()
        self._title = title
        self._run_fn = task
        self._finished = False

    def compose(self) -> ComposeResult:
        yield Static(self._title, id="step")
        yield RichLog(id="log", highlight=True, markup=True, wrap=True)
        yield Static("", id="hint")
        yield Footer()

    def on_mount(self) -> None:
        self.query_one(RichLog).focus()
        self.app.run_worker(lambda: self._run_fn(self), thread=True)

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
        msg = (
            "[green]Done![/green]  Press [bold]Enter[/bold] to close."
            if success
            else "[red]Failed.[/red]  Press [bold]Enter[/bold] to close."
        )
        self.query_one("#hint", Static).update(msg)

    def action_close(self) -> None:
        if self._finished:
            self.dismiss(True)
