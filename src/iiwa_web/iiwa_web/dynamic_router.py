import importlib
import math
import time
from typing import Any

from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from pydantic import Field, create_model

from .config_loader import EndpointDef, FieldDef, load_api_config, load_joint_limits
from .ros_node import get_bridge

_POLL_INTERVAL = 0.05

_TYPE_MAP: dict[str, type] = {
    "string": str,
    "float": float,
    "int": int,
    "bool": bool,
    "float_array": list[float],
}


def _import_ros_type(type_str: str):
    """'sensor_msgs/msg/JointState' → класс JointState."""
    parts = type_str.split("/")
    module = importlib.import_module(".".join(parts[:-1]))
    return getattr(module, parts[-1])


def _to_python(val: Any) -> Any:
    """Конвертирует ROS-значение в JSON-сериализуемый Python-тип."""
    if isinstance(val, float):
        return None if math.isnan(val) else val
    if hasattr(val, "__iter__") and not isinstance(val, (str, bytes)):
        return [None if (isinstance(v, float) and math.isnan(v)) else v for v in val]
    return val


def _extract(msg, fields: list[str]) -> dict:
    return {f: _to_python(getattr(msg, f)) for f in fields}


def _build_model(name: str, field_defs: list[FieldDef]) -> type:
    """Динамически создаёт Pydantic-модель из списка FieldDef."""
    definitions: dict[str, tuple] = {}
    for fd in field_defs:
        py_type = _TYPE_MAP[fd.type]
        kwargs: dict[str, Any] = {"description": fd.description}
        if fd.min is not None:
            kwargs["ge"] = fd.min
        if fd.max is not None:
            kwargs["le"] = fd.max
        if fd.length is not None:
            kwargs["min_length"] = fd.length
            kwargs["max_length"] = fd.length
        kwargs["default"] = ... if fd.required else fd.default
        definitions[fd.name] = (py_type, Field(**kwargs))
    return create_model(name, **definitions)


def _make_topic_handler(ep: EndpointDef):
    ros_type = _import_ros_type(ep.msg_type)
    ros_name = ep.ros_name
    fields = ep.fields
    timeout = ep.timeout

    def handler():
        bridge = get_bridge()
        bridge.subscribe(ros_name, ros_type)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = bridge.get_latest(ros_name)
            if msg is not None:
                return JSONResponse(_extract(msg, fields))
            time.sleep(_POLL_INTERVAL)
        raise HTTPException(503, f"Timeout waiting for topic '{ros_name}'")

    return handler


def _make_service_handler(ep: EndpointDef, Body: type | None):
    ros_type = _import_ros_type(ep.msg_type)
    ros_name = ep.ros_name
    response_fields = ep.response_fields
    timeout = ep.timeout
    field_defs = ep.request_fields

    if Body is None:
        def handler():
            try:
                resp = get_bridge().call_service(ros_type, ros_name, ros_type.Request(), timeout)
            except (RuntimeError, TimeoutError) as e:
                raise HTTPException(503, str(e))
            return _extract(resp, response_fields) if response_fields else {"success": resp.success}
    else:
        def handler(body: Body):  # type: ignore[valid-type]
            req = ros_type.Request()
            for fd in field_defs:
                setattr(req, fd.name, getattr(body, fd.name))
            try:
                resp = get_bridge().call_service(ros_type, ros_name, req, timeout)
            except (RuntimeError, TimeoutError) as e:
                raise HTTPException(503, str(e))
            return _extract(resp, response_fields) if response_fields else {"success": resp.success}

    return handler


def _make_action_handler(ep: EndpointDef, Body: type, joint_limits: list[tuple[float, float]]):
    ros_type = _import_ros_type(ep.msg_type)
    ros_name = ep.ros_name
    response_fields = ep.response_fields
    timeout = ep.timeout
    field_defs = ep.request_fields

    choice_fields = [(fd.name, fd.choices, fd.normalize) for fd in field_defs if fd.choices]
    jl_fields = [fd.name for fd in field_defs if fd.joint_limits]

    def handler(body: Body):  # type: ignore[valid-type]
        values: dict[str, Any] = {fd.name: getattr(body, fd.name) for fd in field_defs}

        # Нормализация и валидация choices
        for fname, choices, normalize in choice_fields:
            val = values[fname]
            if normalize == "lower" and isinstance(val, str):
                val = val.lower()
            elif normalize == "upper" and isinstance(val, str):
                val = val.upper()
            if val not in choices:
                raise HTTPException(422, f"Поле '{fname}' должно быть одним из {choices}, получено '{val}'")
            values[fname] = val

        # Валидация лимитов суставов из joint_limits.yaml
        for fname in jl_fields:
            joints = values[fname]
            if len(joints) != len(joint_limits):
                raise HTTPException(
                    422,
                    f"Ожидалось {len(joint_limits)} суставов, получено {len(joints)}",
                )
            for i, (pos, (lo, hi)) in enumerate(zip(joints, joint_limits)):
                if not (lo <= pos <= hi):
                    raise HTTPException(
                        422,
                        f"Сустав {i + 1}: {pos:.4f} рад вне диапазона [{lo:.3f}, {hi:.3f}]",
                    )

        goal = ros_type.Goal()
        for fd in field_defs:
            setattr(goal, fd.name, values[fd.name])

        try:
            result = get_bridge().send_action(ros_type, ros_name, goal, timeout)
        except (RuntimeError, TimeoutError) as e:
            raise HTTPException(503, str(e))

        return _extract(result.result, response_fields) if response_fields else {}

    return handler


def _quat_to_euler_zyx(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Quaternion → ZYX Euler (KUKA ABC: A=yaw, B=pitch, C=roll)."""
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    return roll, pitch, yaw  # C, B, A


def _make_tf_handler(ep: EndpointDef):
    parent = ep.parent_frame
    child = ep.child_frame
    timeout = ep.timeout

    def handler():
        try:
            tf = get_bridge().lookup_transform(parent, child, timeout)
        except RuntimeError as e:
            raise HTTPException(503, str(e))

        t = tf.transform.translation
        r = tf.transform.rotation
        roll, pitch, yaw = _quat_to_euler_zyx(r.x, r.y, r.z, r.w)

        return {
            "position": {"x": t.x, "y": t.y, "z": t.z},
            "orientation": {
                "quaternion": {"x": r.x, "y": r.y, "z": r.z, "w": r.w},
                "euler_rad": {"a": yaw, "b": pitch, "c": roll},
                "euler_deg": {
                    "a": math.degrees(yaw),
                    "b": math.degrees(pitch),
                    "c": math.degrees(roll),
                },
            },
            "frame": {"parent": parent, "child": child},
        }

    return handler


def build_dynamic_router() -> APIRouter:
    """Читает api_endpoints.yaml и joint_limits.yaml, возвращает готовый APIRouter."""
    endpoints = load_api_config()
    joint_limits = load_joint_limits()

    router = APIRouter()

    for ep in endpoints:
        # Имя модели — CamelCase из пути (/robot/move/joints → RobotMoveJoints)
        model_name = "".join(p.title() for p in ep.path.strip("/").split("/"))

        Body: type | None = None
        if ep.request_fields:
            Body = _build_model(f"{model_name}Request", ep.request_fields)

        if ep.type == "topic":
            handler = _make_topic_handler(ep)
        elif ep.type == "service":
            handler = _make_service_handler(ep, Body)
        elif ep.type == "action":
            if Body is None:
                raise ValueError(f"Action-эндпоинт '{ep.path}' не имеет request_fields")
            handler = _make_action_handler(ep, Body, joint_limits)
        elif ep.type == "tf":
            if not ep.parent_frame or not ep.child_frame:
                raise ValueError(f"TF-эндпоинт '{ep.path}' требует parent_frame и child_frame")
            handler = _make_tf_handler(ep)
        else:
            raise ValueError(f"Неизвестный тип эндпоинта: '{ep.type}'")

        router.add_api_route(
            ep.path,
            handler,
            methods=[ep.method],
            summary=ep.summary or ep.description,
            description=ep.description,
            tags=ep.tags,
            deprecated=ep.deprecated,
        )

    return router
