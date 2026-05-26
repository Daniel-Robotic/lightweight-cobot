import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from sensor_msgs.msg import JointState
from .ros_node import init_ros_node, get_bridge
from .dynamic_router import build_dynamic_router
from . import runner, trajectory


@asynccontextmanager
async def lifespan(_: FastAPI):
    init_ros_node()
    get_bridge().subscribe("/joint_states", JointState)
    yield


app = FastAPI(lifespan=lifespan)
app.include_router(build_dynamic_router())
app.include_router(runner.router)
app.include_router(trajectory.router)


def main():
    uvicorn.run(app, host="localhost", port=8007)

if __name__ == "__main__":
    main()
