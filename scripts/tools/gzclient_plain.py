#!/usr/bin/env python3
"""Launch plain gzclient without the gazebo_ros wrapper's extra system plugins."""

import os
import sys
import time


def _truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def main():
    env = dict(os.environ)

    try:
        delay = float(env.get("GAZEBO_GUI_STARTUP_DELAY_SEC", "0") or "0")
    except ValueError:
        delay = 0.0
    if delay > 0:
        time.sleep(delay)

    if _truthy(env.get("GAZEBO_GUI_SOFTWARE_RENDERING", "0")):
        env["LIBGL_ALWAYS_SOFTWARE"] = "1"
        env["GALLIUM_DRIVER"] = env.get("GALLIUM_DRIVER", "llvmpipe")
        env["MESA_LOADER_DRIVER_OVERRIDE"] = env.get(
            "MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe"
        )

    cmd = [env.get("GAZEBO_GZCLIENT_BIN", "gzclient"), *sys.argv[1:]]
    os.execvpe(cmd[0], cmd, env)


if __name__ == "__main__":
    main()
