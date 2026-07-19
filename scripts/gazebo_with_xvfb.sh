#!/usr/bin/env python3
"""Run gzserver with an owned Xvfb display for headless camera simulation."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def start_xvfb() -> tuple[str, subprocess.Popen]:
    """Claim the first display whose X socket becomes ready."""
    for number in range(99, 110):
        socket_path = Path(f"/tmp/.X11-unix/X{number}")
        if socket_path.exists():
            continue
        display = f":{number}"
        process = subprocess.Popen(
            ["Xvfb", display, "-screen", "0", "1280x1024x24", "-nolisten", "tcp"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                break
            if socket_path.exists():
                return display, process
            time.sleep(0.05)
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=2.0)
            except ProcessLookupError:
                pass
    raise RuntimeError("no free Xvfb display in :99-:109")


def stop_group(process: subprocess.Popen, first_signal: int) -> None:
    if process.poll() is not None:
        return
    try:
        group = os.getpgid(process.pid)
        os.killpg(group, first_signal)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10.0 if first_signal == signal.SIGINT else 2.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(group, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=3.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(group, signal.SIGKILL)
        process.wait(timeout=3.0)
    except ProcessLookupError:
        return


def main() -> int:
    if len(sys.argv) < 2:
        raise RuntimeError("expected gzserver command")

    display, xvfb = start_xvfb()

    environment = os.environ.copy()
    environment["DISPLAY"] = display
    gazebo = subprocess.Popen(sys.argv[1:], env=environment, start_new_session=True)
    stopping = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        if stopping:
            return
        stopping = True
        stop_group(gazebo, signal.SIGTERM)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while gazebo.poll() is None and not stopping:
            time.sleep(0.1)
        return gazebo.wait()
    finally:
        stop_group(gazebo, signal.SIGTERM)
        stop_group(xvfb, signal.SIGTERM)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as error:
        print(f"gazebo_with_xvfb: {error}", file=sys.stderr)
        raise SystemExit(1)
