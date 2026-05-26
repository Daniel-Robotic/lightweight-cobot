import math

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field, field_validator
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from iiwa_msgs.action import MoveToPose, MoveToJoints
from iiwa_msgs.srv import MoveToNamedPose

from .deps import ros_topic
from .ros_node import get_bridge

router = APIRouter(prefix="/robot", tags=["robot"])

JOINT_LIMITS = [
    (-2.967, 2.967),
    (-2.094, 2.094),
    (-2.967, 2.967),
    (-2.094, 2.094),
    (-2.967, 2.967),
    (-2.094, 2.094),
    (-3.054, 3.054),
]

PLANNERS = ["ompl", "ptp", "lin", "circ", "chomp"]


def _clean(values):
    return [None if math.isnan(v) else v for v in values]


def _action_error(e: Exception):
    return HTTPException(status_code=503, detail=str(e))



class MoveToPoseRequest(BaseModel):
    x: float = Field(..., description="Позиция X в метрах")
    y: float = Field(..., description="Позиция Y в метрах")
    z: float = Field(..., description="Позиция Z в метрах")
    a: float = Field(0.0, description="Угол A (ZYX Эйлер, KUKA ABC) в радианах")
    b: float = Field(0.0, description="Угол B в радианах")
    c: float = Field(0.0, description="Угол C в радианах")
    speed: float = Field(0.1, ge=0.01, le=1.0, description="Скорость [0.01 – 1.0]")
    planner: str = Field("ompl", description=f"Планировщик: {', '.join(PLANNERS)}")
    frame_id: str = Field("", description="Целевой фрейм (пусто = default_frame)")

    @field_validator("planner")
    @classmethod
    def check_planner(cls, v: str) -> str:
        if v.lower() not in PLANNERS:
            raise ValueError(f"Неизвестный планировщик '{v}'. Доступные: {', '.join(PLANNERS)}")
        return v.lower()


class MoveToJointsRequest(BaseModel):
    joints: list[float] = Field(
        ..., min_length=7, max_length=7,
        description="Позиции суставов в радианах [j1..j7]",
    )
    speed: float = Field(0.1, ge=0.01, le=1.0, description="Скорость [0.01 – 1.0]")

    @field_validator("joints")
    @classmethod
    def check_limits(cls, joints: list[float]) -> list[float]:
        for i, (pos, (lo, hi)) in enumerate(zip(joints, JOINT_LIMITS)):
            if not (lo <= pos <= hi):
                raise ValueError(
                    f"Сустав {i + 1}: {pos:.4f} рад вне диапазона [{lo:.3f}, {hi:.3f}]"
                )
        return joints


class MoveToNamedRequest(BaseModel):
    name: str = Field(..., description="Имя позиции из SRDF")
    speed: float = Field(0.1, ge=0.01, le=1.0, description="Скорость [0.01 – 1.0]")
    accel_scale: float = Field(0.0, ge=0.0, le=1.0, description="Ускорение (0 = равно speed)")



@router.get("/joint_states", summary="Текущее состояние суставов")
def get_joint_states(msg: JointState = Depends(ros_topic("/joint_states", JointState))):
    return JSONResponse({
        "name": list(msg.name),
        "position": _clean(msg.position),
        "velocity": _clean(msg.velocity),
        "effort": _clean(msg.effort),
    })


@router.post("/move/pose", summary="[Action] Переместить в декартову позу")
def move_to_pose(req: MoveToPoseRequest):
    goal = MoveToPose.Goal()
    goal.x, goal.y, goal.z = req.x, req.y, req.z
    goal.a, goal.b, goal.c = req.a, req.b, req.c
    goal.speed = req.speed
    goal.planner = req.planner
    goal.frame_id = req.frame_id

    try:
        result = get_bridge().send_action(MoveToPose, "cobot/move_to_pose", goal)
    except (RuntimeError, TimeoutError) as e:
        raise _action_error(e)

    return {"success": result.result.success, "message": result.result.message}


@router.post("/move/joints", summary="[Action] Переместить в позиции суставов")
def move_to_joints(req: MoveToJointsRequest):
    goal = MoveToJoints.Goal()
    goal.joints = req.joints
    goal.speed = req.speed

    try:
        result = get_bridge().send_action(MoveToJoints, "cobot/move_to_joints", goal)
    except (RuntimeError, TimeoutError) as e:
        raise _action_error(e)

    return {"success": result.result.success, "message": result.result.message}


@router.post("/move/named", summary="[Service] Переместить в именованную позу из SRDF")
def move_to_named(req: MoveToNamedRequest):
    request = MoveToNamedPose.Request()
    request.name = req.name
    request.speed = req.speed
    request.accel_scale = req.accel_scale

    try:
        response = get_bridge().call_service(MoveToNamedPose, "cobot/move_to_named", request)
    except (RuntimeError, TimeoutError) as e:
        raise HTTPException(status_code=503, detail=str(e))

    return {"success": response.success, "message": response.message}


@router.post("/stop", summary="[Service] Немедленно остановить движение")
def stop():
    try:
        response = get_bridge().call_service(Trigger, "cobot/stop", Trigger.Request())
    except (RuntimeError, TimeoutError) as e:
        raise HTTPException(status_code=503, detail=str(e))

    return {"success": response.success, "message": response.message}
