import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from .ros_node import init_ros_node
from .dynamic_router import build_dynamic_router


@asynccontextmanager
async def lifespan(_: FastAPI):
    init_ros_node()
    yield


app = FastAPI(lifespan=lifespan)
app.include_router(build_dynamic_router())


def main():
    uvicorn.run(app, host="0.0.0.0", port=8007)

if __name__ == "__main__":
    main()
