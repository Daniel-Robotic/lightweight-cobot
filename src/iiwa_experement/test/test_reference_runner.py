"""Trace files and final results are machine-readable and flushed."""
import json
from pathlib import Path

from iiwa_experement.trace import RunResult, TraceWriter


def test_trace_writer_creates_and_flushes_jsonl(tmp_path):
    writer = TraceWriter(7, tmp_path / 'runs', sim_time=lambda: 12.5)
    writer.event({'operation': 'take_tool', 'object': 'vacuum', 'phase': 'start'})
    result = RunResult(
        experiment_id=7,
        status='completed',
        message='ok',
        delivered=[('screw_1', 'hole_1')],
        returned_tools=['magnetic_tool'],
        home_reached=True,
        simulation_time=20.0,
    )
    writer.finish(result)
    writer.close()

    assert writer.path.parent == tmp_path / 'runs'
    assert writer.path.name.startswith('experiment_07_')
    lines = [
        json.loads(line)
        for line in writer.path.read_text(encoding='utf-8').splitlines()
    ]
    assert lines[0]['operation'] == 'take_tool'
    assert lines[0]['simulation_time'] == 12.5
    final = lines[-1]['result']
    assert final['experiment_id'] == 7
    assert final['status'] == 'completed'
    assert final['delivered'] == [{'screw': 'screw_1', 'hole': 'hole_1'}]
    assert final['home_reached'] is True


def test_trace_events_are_flushed_before_close(tmp_path):
    writer = TraceWriter(1, tmp_path, sim_time=lambda: 1.0)
    writer.event({'event': 'one'})
    text = writer.path.read_text(encoding='utf-8')
    assert '"event": "one"' in text
    writer.close()


def test_trace_writer_expands_user_path(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    writer = TraceWriter(2, '~/trace')
    writer.close()
    assert writer.path.parent == tmp_path / 'trace'
