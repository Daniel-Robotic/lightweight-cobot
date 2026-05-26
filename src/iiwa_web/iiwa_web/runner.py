import os
import signal
import subprocess
import threading
import tempfile
from collections import deque
from pathlib import Path
from typing import Optional

import yaml
from fastapi import APIRouter, Form, HTTPException, Query, UploadFile, File
from std_srvs.srv import Trigger

from .ros_node import get_bridge

router = APIRouter(prefix="/sequences", tags=["sequences"])

_UPLOAD_DIR = Path(tempfile.gettempdir()) / "iiwa_configs"
_UPLOAD_DIR.mkdir(exist_ok=True)

_LOG_BUFFER = 300

_process: Optional[subprocess.Popen] = None
_log_lines: deque[str] = deque(maxlen=_LOG_BUFFER)
_lock = threading.Lock()


def _stream_output(proc: subprocess.Popen) -> None:
    for line in proc.stdout:
        _log_lines.append(line.rstrip("\n"))


def _build_cmd(config_path: str, n_iterations: int, delay: float,
               bag_path: str, topics: list[str],
               joints_action: str, pose_action: str) -> list[str]:
    cmd = [
        "ros2", "run", "iiwa_planning", "motion_sequence_runner",
        "--ros-args",
        "-p", f"config_path:={config_path}",
        "-p", f"n_iterations:={n_iterations}",
        "-p", f"delay_between_iterations:={delay}",
        "-p", f"joints_action:={joints_action}",
        "-p", f"pose_action:={pose_action}",
    ]
    if bag_path:
        cmd += ["-p", f"bag_path:={bag_path}"]
    if topics:
        topics_yaml = yaml.dump(topics, default_flow_style=True).strip()
        cmd += ["-p", f"topics:={topics_yaml}"]
    return cmd


@router.post("/start", summary="Загрузить конфиг и запустить motion_sequence_runner")
async def start_runner(
    config: UploadFile = File(..., description="JSON-файл конфигурации последовательности"),
    n_iterations: int = Form(3, ge=1, description="Число повторений"),
    delay_between_iterations: float = Form(5.0, ge=0.0, description="Пауза между итерациями [с]"),
    bag_path: str = Form("", description="Путь для записи rosbag (пусто = не записывать)"),
    topics: str = Form("", description="Топики для bag через запятую (пусто = все)"),
    joints_action: str = Form("cobot/move_to_joints", description="Action для суставного движения"),
    pose_action: str = Form("cobot/move_to_pose", description="Action для декартова движения"),
):
    global _process
    with _lock:
        if _process and _process.poll() is None:
            raise HTTPException(409, f"Runner уже запущен (pid={_process.pid})")

        filename = config.filename or "config.json"
        dest = _UPLOAD_DIR / filename
        dest.write_bytes(await config.read())

        topics_list = [t.strip() for t in topics.split(",") if t.strip()]

        _log_lines.clear()
        cmd = _build_cmd(
            config_path=str(dest),
            n_iterations=n_iterations,
            delay=delay_between_iterations,
            bag_path=bag_path,
            topics=topics_list,
            joints_action=joints_action,
            pose_action=pose_action,
        )

        _process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        threading.Thread(target=_stream_output, args=(_process,), daemon=True).start()

    return {"status": "started", "pid": _process.pid, "config": filename}


@router.post("/stop", summary="Остановить motion_sequence_runner и послать cobot/stop")
def stop_runner():
    global _process
    with _lock:
        if not _process or _process.poll() is not None:
            raise HTTPException(404, "Runner не запущен")
        pgid = os.getpgid(_process.pid)
        os.killpg(pgid, signal.SIGTERM)
        try:
            _process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
            _process.wait()
        code = _process.returncode

    result = get_bridge().call_service(Trigger, "cobot/stop", Trigger.Request())
    return {"status": "stopped", "returncode": code, "success": result.success, "message": result.message}


@router.get("/status", summary="Статус motion_sequence_runner")
def runner_status():
    if not _process:
        return {"status": "idle"}
    code = _process.poll()
    if code is None:
        return {"status": "running", "pid": _process.pid}
    return {"status": "finished", "returncode": code}


@router.get("/logs", summary="Последние строки вывода motion_sequence_runner")
def runner_logs(n: int = Query(50, ge=1, le=_LOG_BUFFER, description="Количество последних строк")):
    lines = list(_log_lines)
    return {"lines": lines[-n:], "total_buffered": len(lines)}
