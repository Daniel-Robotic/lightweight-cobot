import threading
from contextlib import asynccontextmanager

import rclpy
import uvicorn
from fastapi import FastAPI
from sensor_msgs.msg import JointState

from .dynamic_router import build_dynamic_router
from .ros_node import CobotWebNode, get_bridge, set_bridge
from . import runner, trajectory


def main():
    rclpy.init()
    node = CobotWebNode()
    set_bridge(node)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    host = node.get_parameter('host').value
    port = node.get_parameter('port').value
    endpoints_path = node.get_parameter('endpoints_path').value or None
    joint_limits_path = node.get_parameter('joint_limits_path').value or None

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        get_bridge().subscribe("/joint_states", JointState)
        yield

    app = FastAPI(lifespan=lifespan)
    app.include_router(build_dynamic_router(endpoints_path, joint_limits_path))
    app.include_router(runner.router)
    app.include_router(trajectory.router)

    uvicorn.run(app, host=host, port=port)


if __name__ == "__main__":
    main()
