import time
from fastapi import HTTPException
from .ros_node import get_bridge

_TIMEOUT = 2.0
_POLL_INTERVAL = 0.05

def ros_topic(topic_name: str, msg_type):

    def dependency():
        bridge = get_bridge()
        bridge.subscribe(topic_name, msg_type)

        deadline = time.monotonic() + _TIMEOUT
        while time.monotonic() < deadline:
            msg = bridge.get_latest(topic_name)
            if msg is not None:
                return msg
            time.sleep(_POLL_INTERVAL)

        raise HTTPException(
            status_code=503,
            detail=f"Timeout waiting for message on topic '{topic_name}'.",
        )

    return dependency