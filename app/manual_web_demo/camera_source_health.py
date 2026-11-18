"""Validate the HTTP writer's independent heartbeat before releasing ROS demand."""
import json
import math
from pathlib import Path


def fresh_http_camera(path, *, latest_path, boot_id, now, max_age):
    try:
        status = json.loads(Path(path).read_text(encoding="utf-8"))
        received = status.get("received_at")
        if (not boot_id or status.get("owner_boot_id") != boot_id
                or status.get("source") != "robot_http_mjpeg"
                or status.get("available") is not True
                or isinstance(received, bool) or not isinstance(received, (int, float))
                or not math.isfinite(received) or not 0 <= now - received <= max_age
                or status.get("latest_path") != str(Path(latest_path).resolve())
                or not Path(latest_path).is_file()
                or any(type(status.get(k)) is not int or status[k] <= 0
                       for k in ("width", "height"))):
            return None
        return status
    except (OSError, ValueError, TypeError, AttributeError):
        return None
