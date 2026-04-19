#!/usr/bin/env python3
"""Fail fast when simulation prerequisites are not satisfied."""

import os
import socket
import subprocess
import sys
import time
from typing import List, Set

import rospy
import rosnode
from gazebo_msgs.srv import GetWorldProperties


def port_in_use(port: int, host: str = "127.0.0.1") -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.15)
    try:
        return sock.connect_ex((host, int(port))) == 0
    finally:
        sock.close()


def pids_listening_on_port(port: int) -> Set[int]:
    """Return local process IDs listening on a TCP port."""
    try:
        output = subprocess.check_output(
            ["ss", "-ltnp"], stderr=subprocess.DEVNULL, text=True
        )
    except Exception:
        return set()

    target = f":{int(port)}"
    pids: Set[int] = set()
    for line in output.splitlines():
        if target not in line:
            continue
        for part in line.split("pid=")[1:]:
            pid_text = ""
            for ch in part:
                if ch.isdigit():
                    pid_text += ch
                else:
                    break
            if pid_text:
                try:
                    pids.add(int(pid_text))
                except ValueError:
                    continue
    return pids


def terminate_pids(pids: Set[int], term_timeout_sec: float = 1.0) -> Set[int]:
    """Best-effort SIGTERM then SIGKILL for the provided process IDs."""
    alive: Set[int] = set()
    for pid in sorted(pids):
        if pid <= 1 or pid == os.getpid():
            continue
        try:
            os.kill(pid, 15)  # SIGTERM
            alive.add(pid)
        except ProcessLookupError:
            continue
        except PermissionError:
            continue

    deadline = time.time() + max(0.0, term_timeout_sec)
    while alive and time.time() < deadline:
        still_alive = set()
        for pid in alive:
            try:
                os.kill(pid, 0)
                still_alive.add(pid)
            except ProcessLookupError:
                continue
            except PermissionError:
                still_alive.add(pid)
        if not still_alive:
            return set()
        alive = still_alive
        time.sleep(0.05)

    # Escalate stubborn processes.
    for pid in sorted(alive):
        try:
            os.kill(pid, 9)  # SIGKILL
        except ProcessLookupError:
            continue
        except PermissionError:
            continue
    return alive


def cleanup_ros_nodes(node_names: List[str]) -> List[str]:
    """Shutdown lingering nodes from prior sessions by canonical name."""
    try:
        active_nodes = set(rosnode.get_node_names())
    except Exception:
        return []

    targets = [name for name in node_names if name in active_nodes]
    if not targets:
        return []
    try:
        rosnode.kill_nodes(targets)
    except Exception:
        pass
    return targets


def main():
    rospy.init_node("simulation_preflight", anonymous=False)

    gazebo_master_port = int(rospy.get_param("~gazebo_master_port", 11345))
    allow_existing_gazebo = bool(rospy.get_param("~allow_existing_gazebo", False))
    required_free_ports = list(rospy.get_param("~required_free_ports", []))
    auto_cleanup = bool(rospy.get_param("~auto_cleanup", True))
    stale_node_names = list(rospy.get_param("~stale_node_names", []))

    errors = []
    cleanup_actions = []

    if not allow_existing_gazebo and port_in_use(gazebo_master_port):
        errors.append(f"Gazebo master port {gazebo_master_port} is already in use.")
        try:
            rospy.wait_for_service("/gazebo/get_world_properties", timeout=0.5)
            get_world_properties = rospy.ServiceProxy(
                "/gazebo/get_world_properties", GetWorldProperties
            )
            world = get_world_properties()
            model_names = list(world.model_names)
            if "heron" in model_names:
                errors.append("Existing Gazebo world already contains model 'heron'.")
        except Exception:
            pass

    for port in required_free_ports:
        port = int(port)
        if port_in_use(port):
            errors.append(f"Required TCP port {port} is already in use.")

    if errors and auto_cleanup:
        cleanup_ports = [gazebo_master_port] + [int(port) for port in required_free_ports]
        cleanup_pids: Set[int] = set()
        for port in cleanup_ports:
            cleanup_pids.update(pids_listening_on_port(port))
        if cleanup_pids:
            cleanup_actions.append(f"terminating pids={sorted(cleanup_pids)}")
            terminate_pids(cleanup_pids)

        cleaned_nodes = cleanup_ros_nodes(stale_node_names)
        if cleaned_nodes:
            cleanup_actions.append(f"killed_nodes={cleaned_nodes}")

        # Re-evaluate after cleanup attempt.
        errors = []
        if not allow_existing_gazebo and port_in_use(gazebo_master_port):
            errors.append(f"Gazebo master port {gazebo_master_port} is already in use.")
        for port in required_free_ports:
            port = int(port)
            if port_in_use(port):
                errors.append(f"Required TCP port {port} is already in use.")

    if errors:
        if cleanup_actions:
            rospy.logwarn("[sim_preflight] Cleanup attempted: %s", "; ".join(cleanup_actions))
        for err in errors:
            rospy.logerr("[sim_preflight] %s", err)
        rospy.logerr(
            "[sim_preflight] Session is still dirty after cleanup; run "
            "`rosnode cleanup`, kill stale launch processes, then relaunch."
        )
        sys.exit(1)

    if cleanup_actions:
        rospy.logwarn("[sim_preflight] Cleaned stale runtime state: %s", "; ".join(cleanup_actions))
    rospy.loginfo("[sim_preflight] Environment looks clean.")
    rospy.loginfo("[sim_preflight] Standing by.")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
