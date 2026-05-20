from __future__ import annotations

from typing import List, Optional

from textual import on
from textual.app import ComposeResult
from textual.binding import Binding
from textual.screen import Screen
from textual.widgets import Footer, Input, RadioButton, RadioSet, Static

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
RadioSet {
    height: auto;
    border: none;
    padding: 0;
    margin-bottom: 1;
}
Input {
    margin-bottom: 1;
}
"""


class PickScreen(Screen[Optional[str]]):
    BINDINGS = [
        Binding("enter", "submit", "Confirm", priority=True),
        Binding("escape", "abort", "Cancel"),
    ]

    def __init__(self, step: str, question: str, options: List[str], default: str):
        super().__init__()
        self._step = step
        self._question = question
        self._options = options
        self._default = default

    def compose(self) -> ComposeResult:
        yield Static(self._step, id="step")
        yield Static(self._question, id="question")
        with RadioSet(id="choices"):
            for opt in self._options:
                yield RadioButton(opt, value=(opt == self._default))
        yield Footer()

    def on_mount(self) -> None:
        self.query_one(RadioSet).focus()

    def action_submit(self) -> None:
        btn = self.query_one("#choices", RadioSet).pressed_button
        self.dismiss(str(btn.label) if btn else self._default)

    def action_abort(self) -> None:
        self.app.exit(None)


class InputScreen(Screen[Optional[str]]):
    BINDINGS = [
        Binding("enter", "submit", "Confirm", priority=True),
        Binding("escape", "abort", "Cancel"),
    ]

    def __init__(self, step: str, question: str, default: str):
        super().__init__()
        self._step = step
        self._question = question
        self._default = default

    def compose(self) -> ComposeResult:
        yield Static(self._step, id="step")
        yield Static(self._question, id="question")
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
