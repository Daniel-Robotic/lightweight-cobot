import csv
import io
import math
import threading
import time
import yaml
from pathlib import Path
from iiwa_planning.direct_trajectory import prepare_trajectory
from collections import deque
from datetime import datetime
from builtin_interfaces.msg import Duration
from fastapi import APIRouter, File, HTTPException, Query, UploadFile
from pydantic import BaseModel, Field
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .config_loader import load_joint_limits, load_joint_names, _resolve_path
from .ros_node import get_bridge

router = APIRouter(prefix="/trajectory", tags=["trajectory"])

TOPIC = "/iiwa_arm_controller/joint_trajectory"
JOINT_NAMES = load_joint_names()
N_JOINTS = len(JOINT_NAMES)
_prepare_lock = threading.Lock()
_admission_lock = threading.Lock()
_generation = 0
_direct_until = 0.0

_log_lines: deque[str] = deque(maxlen=300)


def _log(msg: str) -> None:
    _log_lines.append(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}")


def _to_duration(seconds: float) -> Duration:
    total_ns = round(seconds * 1e9)
    sec, nanosec = divmod(total_ns, 1_000_000_000)
    return Duration(sec=sec, nanosec=nanosec)


def _validate_limits(points: list[list[float]]) -> None:
    limits = load_joint_limits(path=get_bridge().get_parameter("joint_limits_path").value or None)
    for row_idx, positions in enumerate(points):
        for j, (pos, (lo, hi)) in enumerate(zip(positions, limits)):
            if not (lo <= pos <= hi):
                raise HTTPException(
                    422,
                    f"Точка {row_idx + 1}, сустав {j + 1}: "
                    f"{pos:.4f} рад вне диапазона [{lo:.3f}, {hi:.3f}]",
                )


def _build_msg(rows: list[tuple[list[float], float]]) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = JOINT_NAMES
    for positions, t in rows:
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = _to_duration(t)
        msg.points.append(pt)
    return msg


def _stationary_state():
    bridge = get_bridge()
    state, age = bridge.get_latest_with_age('/joint_states')
    timeout = bridge.get_parameter('trajectory_state_timeout').value
    tolerance = bridge.get_parameter('trajectory_start_tolerance').value
    stopped = bridge.get_parameter('trajectory_stopped_velocity').value
    if any(not math.isfinite(x) or x <= 0 for x in (timeout, tolerance, stopped)):
        raise HTTPException(503, 'Invalid direct trajectory monitoring parameters')
    if state is None or age > timeout:
        raise HTTPException(409, 'Fresh joint_states required before sending a trajectory')
    stamp = state.header.stamp.sec + state.header.stamp.nanosec * 1e-9
    source_age = bridge.get_clock().now().nanoseconds * 1e-9 - stamp
    if not math.isfinite(source_age) or not 0 <= source_age <= timeout:
        raise HTTPException(409, 'joint_states timestamp is stale or in the future')
    try:
        indexes = [list(state.name).index(name) for name in JOINT_NAMES]
        current = [state.position[i] for i in indexes]
        velocity = [state.velocity[i] for i in indexes]
    except (ValueError, IndexError):
        raise HTTPException(409, 'Complete named position and velocity feedback required')
    if any(not math.isfinite(x) for x in current + velocity) or any(abs(v) > stopped for v in velocity):
        raise HTTPException(409, 'Direct trajectories must start from a stationary robot')
    return current, tolerance


def _build_safe_msg(rows):
    bridge = get_bridge()
    current, tolerance = _stationary_state()
    path = bridge.get_parameter('joint_limits_path').value
    resolved = Path(path) if path else _resolve_path('iiwa_config', 'config/moveit/joint_limits.yaml')
    data = yaml.safe_load(resolved.read_text())['joint_limits']
    caps = []
    for name in JOINT_NAMES:
        j = data[name]
        caps.append([max(j['min_position'], j.get('soft_min_position', j['min_position'])),
                     min(j['max_position'], j.get('soft_max_position', j['max_position'])),
                     j['max_velocity'], j['max_acceleration'], j['max_jerk']])
    rows = [(list(q), float(t)) for q, t in rows]
    if rows[0][1] > 0:
        rows.insert(0, (current, 0.0))
    elif rows[0][1] == 0 and max(abs(a-b) for a,b in zip(rows[0][0],current)) > tolerance:
        raise HTTPException(422, 'A waypoint at t=0 must match the current joint positions')
    else:
        rows[0] = (current, rows[0][1])
    try:
        times, positions, velocities, accelerations, scale = prepare_trajectory(
            [t for _, t in rows], [q for q, _ in rows], caps)
    except (ValueError, RuntimeError) as exc:
        raise HTTPException(422, str(exc)) from exc
    msg = _build_msg(list(zip(positions, times)))
    for point, v, a in zip(msg.points, velocities, accelerations):
        point.velocities = v
        point.accelerations = a
    return msg, times[-1], scale


def _send_rows(rows):
    global _direct_until
    with _prepare_lock:
        with _admission_lock:
            generation = _generation
            if time.monotonic() < _direct_until:
                raise HTTPException(409, 'A direct trajectory is still active; stop it before replacing it')
        msg, duration, scale = _build_safe_msg(rows)
        with _admission_lock:
            if generation != _generation:
                raise HTTPException(409, 'Trajectory preparation was canceled by stop')
            current, tolerance = _stationary_state()
            if max(abs(a-b) for a,b in zip(current, msg.points[0].positions)) > tolerance:
                raise HTTPException(409, 'Robot moved during trajectory preparation; retry from its new state')
            _publish(msg)
            _direct_until = time.monotonic() + duration + get_bridge().get_parameter('trajectory_state_timeout').value
        return msg, duration, scale


def _publish(msg: JointTrajectory) -> None:
    get_bridge().publish(TOPIC, JointTrajectory, msg)


class Waypoint(BaseModel):
    positions: list[float] = Field(
        ..., min_length=N_JOINTS, max_length=N_JOINTS,
        description="Позиции суставов [j1..j7] в радианах",
    )
    time_from_start: float = Field(..., ge=0.0, description="Время от начала траектории [с]")


class SendRequest(BaseModel):
    points: list[Waypoint] = Field(..., min_length=1, description="Точки траектории")
    validate_limits: bool = Field(True, description="Проверять лимиты суставов")


@router.post("/send", summary="Отправить траекторию вручную (JSON)")
def send_trajectory(req: SendRequest):
    """
    Принимает список точек с позициями суставов и временем от начала.
    Публикует `JointTrajectory` в `/iiwa_arm_controller/joint_trajectory`.
    """
    rows = [(wp.positions, wp.time_from_start) for wp in req.points]

    if req.validate_limits:
        _validate_limits([r[0] for r in rows])

    msg, duration, time_scale = _send_rows(rows)
    _log(f"[send] {len(rows)} точек, t_end={rows[-1][1]:.2f}с")
    return {"status": "sent", "points": len(msg.points), "duration": duration, "time_scale": time_scale}


@router.post("/send_csv", summary="Загрузить CSV и отправить траекторию")
async def send_csv_trajectory(
    file: UploadFile = File(
        ...,
        description="CSV с заголовком. Колонки суставов: joint_1..joint_7 (или joint1..joint7). Колонка времени: t.",
    ),
    separator: str = Query(",", description="Разделитель колонок (например: ',' ';' '\\t')"),
    validate_limits: bool = Query(True, description="Проверять лимиты суставов"),
):
    """
    Ожидаемый формат (первая строка — обязательный заголовок):

        joint1,joint2,joint3,joint4,joint5,joint6,joint7,t
        -2.55,-0.71,-0.77,0.028,0.0,-2.09,-0.10,0.0
        -2.54,-0.71,-0.77,0.029,0.0,-2.09,-0.10,0.01

    Порядок и имена колонок произвольны — сопоставление идёт по заголовку.
    Имена суставов нормализуются: `joint_1` = `joint1` = `JOINT1`.
    Колонка времени определяется по заголовку `t`, `time` или `time_from_start`.
    """
    sep = separator.replace("\\t", "\t")
    content = (await file.read()).decode("utf-8")
    reader = csv.reader(io.StringIO(content), delimiter=sep)

    try:
        raw_headers = next(reader)
    except StopIteration:
        raise HTTPException(422, "Файл пуст")

    headers = [h.strip() for h in raw_headers]

    def _norm(s: str) -> str:
        return s.lower().replace("_", "").replace(" ", "")

    TIME_ALIASES = {"t", "time", "timefromstart"}
    norm_joint_to_idx = {_norm(j): i for i, j in enumerate(JOINT_NAMES)}

    col_joint: dict[int, int] = {}  # col_index -> joint_index
    col_time: int | None = None

    for col_idx, h in enumerate(headers):
        n = _norm(h)
        if n in TIME_ALIASES:
            col_time = col_idx
        elif n in norm_joint_to_idx:
            col_joint[col_idx] = norm_joint_to_idx[n]

    if col_time is None:
        raise HTTPException(422, f"Колонка времени не найдена. Ожидалось одно из: t, time, time_from_start. Заголовки: {headers}")

    missing = sorted(set(range(N_JOINTS)) - set(col_joint.values()))
    if missing:
        raise HTTPException(422, f"Не найдены колонки для суставов: {[JOINT_NAMES[i] for i in missing]}")

    joint_to_col = {j_idx: c_idx for c_idx, j_idx in col_joint.items()}

    rows: list[tuple[list[float], float]] = []
    for line_no, row in enumerate(reader, start=2):
        row = [c.strip() for c in row]
        if not any(row):
            continue
        if len(row) != len(headers):
            raise HTTPException(
                422,
                f"Строка {line_no}: ожидалось {len(headers)} столбцов, получено {len(row)}",
            )
        try:
            positions = [float(row[joint_to_col[i]]) for i in range(N_JOINTS)]
            t = float(row[col_time])
        except ValueError as e:
            raise HTTPException(422, f"Строка {line_no}: не удалось распарсить число — {e}")
        if t < 0:
            raise HTTPException(422, f"Строка {line_no}: t не может быть отрицательным")
        rows.append((positions, t))

    if not rows:
        raise HTTPException(422, "CSV не содержит точек траектории")

    if validate_limits:
        _validate_limits([r[0] for r in rows])

    msg, duration, time_scale = _send_rows(rows)
    _log(f"[csv] {file.filename} → {len(rows)} точек, t_end={rows[-1][1]:.2f}с")
    return {"status": "sent", "points": len(msg.points), "filename": file.filename, "duration": duration, "time_scale": time_scale}


def send_stop_trajectory() -> None:
    """Cancel preparation and publish hold; malformed feedback falls back to cancellation."""
    global _generation, _direct_until
    with _admission_lock:
        _generation += 1
        _direct_until = 0.0
        bridge = get_bridge()
        state = bridge.get_latest('/joint_states')
        try:
            current = [state.position[list(state.name).index(name)] for name in JOINT_NAMES]
            if not all(math.isfinite(x) for x in current):
                raise ValueError('Non-finite feedback')
            _publish(_build_msg([(current, 0.5)]))
            _log('[stop] отправлена точка удержания текущей позиции')
        except (AttributeError, ValueError, IndexError):
            msg = JointTrajectory()
            msg.joint_names = JOINT_NAMES
            _publish(msg)
            _log('[stop] отправлена пустая траектория: неполная обратная связь')


@router.get("/logs", summary="Последние лог-записи траекторного модуля")
def trajectory_logs(n: int = Query(50, ge=1, le=300, description="Количество последних строк")):
    lines = list(_log_lines)
    return {"lines": lines[-n:], "total_buffered": len(lines)}
