import csv
import io
from collections import deque
from datetime import datetime
from builtin_interfaces.msg import Duration
from fastapi import APIRouter, File, HTTPException, Query, UploadFile
from pydantic import BaseModel, Field
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .config_loader import load_joint_limits, load_joint_names
from .ros_node import get_bridge

router = APIRouter(prefix="/trajectory", tags=["trajectory"])

TOPIC = "/iiwa_arm_controller/joint_trajectory"
JOINT_NAMES = load_joint_names()
N_JOINTS = len(JOINT_NAMES)

_log_lines: deque[str] = deque(maxlen=300)


def _log(msg: str) -> None:
    _log_lines.append(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}")


def _to_duration(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    return Duration(sec=sec, nanosec=nanosec)


def _validate_limits(points: list[list[float]]) -> None:
    limits = load_joint_limits()
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

    _publish(_build_msg(rows))
    _log(f"[send] {len(rows)} точек, t_end={rows[-1][1]:.2f}с")
    return {"status": "sent", "points": len(rows)}


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

    _publish(_build_msg(rows))
    _log(f"[csv] {file.filename} → {len(rows)} точек, t_end={rows[-1][1]:.2f}с")
    return {"status": "sent", "points": len(rows), "filename": file.filename}


@router.post("/stop", summary="Остановить выполнение траектории")
def stop_trajectory():
    bridge = get_bridge()

    # Replace ongoing trajectory with single point at current position
    joint_states = bridge.get_latest("/joint_states")
    if joint_states is not None and len(joint_states.position) >= N_JOINTS:
        current_positions = list(joint_states.position[:N_JOINTS])
        hold_msg = _build_msg([(current_positions, 0.5)])
        _publish(hold_msg)
        _log("[stop] отправлена точка удержания текущей позиции")
    else:
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        _publish(msg)
        _log("[stop] joint_states недоступны, отправлена пустая траектория")

    # cobot/stop cancels MoveIt action-based motion
    result = bridge.call_service(Trigger, "cobot/stop", Trigger.Request())
    _log(f"[stop] cobot/stop -> success={result.success}, message={result.message}")
    return {"status": "stopped", "success": result.success, "message": result.message}


@router.get("/logs", summary="Последние лог-записи траекторного модуля")
def trajectory_logs(n: int = Query(50, ge=1, le=300, description="Количество последних строк")):
    lines = list(_log_lines)
    return {"lines": lines[-n:], "total_buffered": len(lines)}
