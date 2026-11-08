"""Machine-readable JSONL trace for one reference run."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


@dataclass
class RunResult:
    experiment_id: int
    status: str
    message: str
    events: list[dict[str, Any]] = field(default_factory=list)
    delivered: list[tuple[str, str]] = field(default_factory=list)
    cap: dict[str, Any] | None = None
    sealant: dict[str, Any] | None = None
    returned_tools: list[str] = field(default_factory=list)
    home_reached: bool = False
    simulation_time: float | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            'experiment_id': self.experiment_id,
            'status': self.status,
            'message': self.message,
            'delivered': [
                {'screw': screw, 'hole': hole} for screw, hole in self.delivered
            ],
            'cap': self.cap,
            'sealant': self.sealant,
            'returned_tools': list(self.returned_tools),
            'home_reached': self.home_reached,
            'simulation_time': self.simulation_time,
        }


class TraceWriter:
    """Write one JSON object per line and flush after every event."""

    def __init__(
        self,
        experiment_id: int,
        trace_dir: str | Path,
        *,
        sim_time: Callable[[], float] | None = None,
    ) -> None:
        directory = Path(str(trace_dir)).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.path = directory / f'experiment_{experiment_id:02d}_{stamp}.jsonl'
        self._handle = self.path.open('w', encoding='utf-8')
        self._sim_time = sim_time

    def event(self, event: dict[str, Any]) -> None:
        payload = dict(event)
        if self._sim_time is not None:
            payload.setdefault('simulation_time', float(self._sim_time()))
        self._handle.write(json.dumps(payload, sort_keys=True) + '\n')
        self._handle.flush()

    def finish(self, result: RunResult) -> None:
        self.event({'event': 'result', 'result': result.as_dict()})
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()

    def __enter__(self) -> 'TraceWriter':
        return self

    def __exit__(self, *exc) -> None:
        self.close()
