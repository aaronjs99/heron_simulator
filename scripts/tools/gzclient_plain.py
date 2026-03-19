#!/usr/bin/env python3
"""Launch plain gzclient without the gazebo_ros wrapper's extra system plugins.

If hardware GLX initialization fails quickly, retry once with software rendering
so the sim core can keep running and the user still gets a GUI.
"""

import os
import subprocess
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

    cmd = [env.get("GAZEBO_GZCLIENT_BIN", "gzclient"), *sys.argv[1:]]

    def _enable_software_rendering(target_env):
        target_env["LIBGL_ALWAYS_SOFTWARE"] = "1"
        target_env["GALLIUM_DRIVER"] = target_env.get("GALLIUM_DRIVER", "llvmpipe")
        target_env["MESA_LOADER_DRIVER_OVERRIDE"] = target_env.get(
            "MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe"
        )

    software_requested = _truthy(env.get("GAZEBO_GUI_SOFTWARE_RENDERING", "0"))
    if software_requested:
        _enable_software_rendering(env)

    try:
        proc = subprocess.run(cmd, env=env)
    except KeyboardInterrupt:
        sys.exit(130)
    if proc.returncode == 0:
        sys.exit(0)

    auto_fallback = _truthy(env.get("GAZEBO_GUI_AUTO_FALLBACK", "1"))
    if software_requested or not auto_fallback:
        sys.exit(proc.returncode)

    fallback_env = dict(env)
    _enable_software_rendering(fallback_env)
    print(
        "gzclient_plain: hardware GUI exited with code "
        f"{proc.returncode}; retrying with software rendering",
        file=sys.stderr,
        flush=True,
    )
    try:
        fallback = subprocess.run(cmd, env=fallback_env)
    except KeyboardInterrupt:
        sys.exit(130)
    sys.exit(fallback.returncode)


if __name__ == "__main__":
    main()
