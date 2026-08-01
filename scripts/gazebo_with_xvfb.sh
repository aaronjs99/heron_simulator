#!/usr/bin/env python3
"""Run gzserver with an owned Xvfb display for headless camera simulation."""

from __future__ import annotations

import os
import signal
import socket
import stat
import subprocess
import sys
import time
from pathlib import Path


def start_xvfb() -> tuple[str, subprocess.Popen]:
    """Claim the first display whose X socket becomes ready."""
    socket_directory = Path("/tmp/.X11-unix")
    unix_transport_available = bool(
        socket_directory.exists() and socket_directory.stat().st_mode & stat.S_ISVTX
    )
    for number in range(99, 110):
        socket_path = Path(f"/tmp/.X11-unix/X{number}")
        if socket_path.exists():
            continue
        display = f":{number}" if unix_transport_available else f"127.0.0.1:{number}"
        transport_args = (
            ["-nolisten", "tcp"]
            if unix_transport_available
            else ["-nolisten", "unix", "-listen", "tcp", "-ac"]
        )
        process = subprocess.Popen(
            ["Xvfb", f":{number}", "-screen", "0", "1280x1024x24"] + transport_args,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                break
            if unix_transport_available and socket_path.exists():
                return display, process
            if not unix_transport_available:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                    probe.settimeout(0.05)
                    if probe.connect_ex(("127.0.0.1", 6000 + number)) == 0:
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
