import threading
from contextlib import asynccontextmanager

import rclpy
import uvicorn
from fastapi import FastAPI, APIRouter
from fastmcp import FastMCP
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .dynamic_router import build_dynamic_router
from .ros_node import CobotWebNode, get_bridge, set_bridge
from . import runner, trajectory, positions, planner


stop_router = APIRouter(tags=["stop"])

@stop_router.post("/stop", summary="Остановить всё: runner, траекторию и планировщик")
def stop_all():
    planner.cancel_active_run()
    runner.stop_if_running()
    trajectory.send_stop_trajectory()

    try:
        result = get_bridge().call_service(
            Trigger,
            "cobot/stop",
            Trigger.Request()
        )

        return {
            "status": "stopped",
            "success": result.success,
            "message": result.message,
        }

    except RuntimeError:
        return {
            "status": "stopped",
            "success": True,
            "message": "Планировщик не запущен",
        }


def add_application_routes(app: FastAPI, endpoints_path, joint_limits_path) -> None:
    """Add the same routers to the schema app and the serving app."""
    app.include_router(build_dynamic_router(endpoints_path, joint_limits_path))
    app.include_router(runner.router)
    app.include_router(trajectory.router)
    app.include_router(positions.router)
    app.include_router(planner.router)
    app.include_router(stop_router)


def build_schema_app(endpoints_path=None, joint_limits_path=None) -> FastAPI:
    schema_app = FastAPI()
    add_application_routes(schema_app, endpoints_path, joint_limits_path)
    return schema_app


def main():
    rclpy.init()
    node = CobotWebNode()
    set_bridge(node)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    host = node.get_parameter('host').value
    port = node.get_parameter('port').value
    endpoints_path = node.get_parameter('endpoints_path').value or None
    joint_limits_path = node.get_parameter('joint_limits_path').value or None

    positions.init()

    _schema_app = build_schema_app(endpoints_path, joint_limits_path)

    mcp = FastMCP.from_fastapi(app=_schema_app)
    mcp_http = mcp.http_app(path='/')

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        async with mcp_http.router.lifespan_context(_):
            get_bridge().subscribe("/joint_states", JointState)
            yield

    app = FastAPI(lifespan=lifespan)
    add_application_routes(app, endpoints_path, joint_limits_path)
    app.mount("/mcp", mcp_http)

    uvicorn.run(app, host=host, port=port)


if __name__ == "__main__":
    main()
