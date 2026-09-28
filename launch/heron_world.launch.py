"""Launch Heron in Gazebo Harmonic with explicit ROS 2 bridges."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.actions import OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _enabled(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch(context, *args, **kwargs):
    values = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "config",
            "sensor_frames_file",
            "world",
            "world_name",
            "gui",
            "headless_rendering",
            "paused",
            "namespace",
            "x",
            "y",
            "z",
            "roll",
            "pitch",
            "yaw",
            "drive_topic",
            "left_thruster_topic",
            "right_thruster_topic",
            "thruster_dynamics_config",
            "propulsion_adapter_enabled",
            "simulated_housekeeping_enabled",
            "use_ig_timing",
            "use_joint_state_fallback",
            "simulate_multibeam_raw",
            "simulate_ping360",
            "sonar_ray_topic",
            "sonar_raw_topic",
            "ping360_ray_topic",
            "sonar_profile_topic",
            "dt100_extrinsic_revision",
            "ping360_extrinsic_revision",
            "sonar_min_range_m",
            "sonar_max_range_m",
            "sense_topic",
            "sense_rate_hz",
            "sense_battery_v",
            "vehicle_battery_topic",
            "payload_battery_topic",
            "status_topic",
            "status_rate_hz",
            "pps_time_topic",
            "camera_time_topic",
            "imu_time_topic",
            "imu_topic",
            "pps_rate_hz",
            "camera_time_image_topics",
            "disabled_sensor_ids",
            "lidar_update_rate_hz",
            "lidar_horizontal_samples",
            "lidar_vertical_samples",
            "hydrodynamics_override_enabled",
            "base_mass_kg",
            "fluid_density_kg_m3",
            "linear_damping_x",
            "quadratic_damping_x",
        )
    }

    simulator_share = Path(get_package_share_directory("heron_simulator"))
    heron_share = Path(get_package_share_directory("heron_description"))
    ig_handle_share = Path(get_package_share_directory("ig_handle"))
    world_path = Path(values["world"]).expanduser()
    if not world_path.is_absolute():
        world_path = simulator_share / "worlds" / world_path
    world_path = world_path.resolve()
    if not world_path.is_file():
        raise RuntimeError(f"Gazebo world does not exist: {world_path}")
    if not values["world_name"].strip():
        values["world_name"] = world_path.stem

    config_path = (heron_share / "urdf" / "configs" / values["config"]).resolve()
    if not config_path.is_file():
        raise RuntimeError(f"Heron simulation profile does not exist: {config_path}")
    sensor_frames = values["sensor_frames_file"].strip()
    if not sensor_frames:
        sensor_frames = str(
            ig_handle_share / "config" / "sensors" / "platform" / "sensor_frames.yaml"
        )
    sensor_frames_path = Path(sensor_frames).expanduser().resolve()
    if not sensor_frames_path.is_file():
        raise RuntimeError(
            f"sensor frame configuration does not exist: {sensor_frames_path}"
        )
    dynamics_config = values["thruster_dynamics_config"].strip()
    dynamics_config_path = (
        Path(dynamics_config).expanduser().resolve()
        if dynamics_config
        else simulator_share / "config" / "dynamics" / "thrusters.yaml"
    )
    if not dynamics_config_path.is_file():
        raise RuntimeError(
            f"thruster dynamics configuration does not exist: {dynamics_config_path}"
        )

    namespace = values["namespace"].strip().strip("/")
    suffix_ns = f"{namespace}/" if namespace else ""
    model_name = f"{namespace.replace('/', '_')}heron" if namespace else "heron"
    disabled = {
        part.strip()
        for part in values["disabled_sensor_ids"].split(",")
        if part.strip()
    }
    environment = os.environ.copy()
    environment["HERON_SENSOR_FRAMES_FILE"] = str(sensor_frames_path)
    resource_paths = [
        str(simulator_share / "models"),
        str(heron_share.parent),
        *environment.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep),
    ]
    system_plugin_paths = [
        str(Path(get_package_prefix("heron_simulator")) / "lib" / "heron_simulator"),
        *environment.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "").split(os.pathsep),
    ]
    environment["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        dict.fromkeys(path for path in resource_paths if path)
    )
    environment["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
        dict.fromkeys(path for path in system_plugin_paths if path)
    )

    profile_runner = (
        Path(get_package_prefix("heron_simulator"))
        / "lib"
        / "heron_simulator"
        / "profile_env_run.sh"
    )
    xacro = shutil.which("xacro")
    if not xacro:
        raise RuntimeError(
            "xacro executable is not available in the sourced ROS 2 environment"
        )
    xacro_args = [
        f"HERON_SIM_MULTIBEAM_ENABLED={values['simulate_multibeam_raw']}",
        f"HERON_SIM_PING360_ENABLED={values['simulate_ping360']}",
        f"HERON_SIM_SONAR_RAY_TOPIC={values['sonar_ray_topic']}",
        f"HERON_SIM_PING360_RAY_TOPIC={values['ping360_ray_topic']}",
        f"HERON_SIM_LIDAR_UPDATE_RATE_HZ={values['lidar_update_rate_hz']}",
        f"HERON_SIM_LIDAR_HORIZONTAL_SAMPLES={values['lidar_horizontal_samples']}",
        f"HERON_SIM_LIDAR_VERTICAL_SAMPLES={values['lidar_vertical_samples']}",
        f"HERON_LIDAR_V={int('3' not in disabled)}",
    ]
    for sensor_id, camera in (("4", "F1"), ("5", "F2"), ("6", "F3"), ("7", "F4")):
        if sensor_id in disabled:
            xacro_args.append(f"HERON_CAMERA_{camera}=0")
    xacro_command = [
        str(profile_runner),
        str(config_path),
        values["hydrodynamics_override_enabled"],
        values["base_mass_kg"],
        values["fluid_density_kg_m3"],
        values["linear_damping_x"],
        values["quadratic_damping_x"],
        "env",
        *xacro_args,
        xacro,
        str(heron_share / "urdf" / "heron.urdf.xacro"),
        "debug:=0",
        "simulation:=true",
        f"namespace:={namespace}",
        f"suffix_ns:={suffix_ns}",
    ]
    rendered = subprocess.run(
        xacro_command,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    if rendered.returncode:
        detail = rendered.stderr.strip() or rendered.stdout.strip()
        raise RuntimeError(f"xacro could not render the Heron model: {detail}")
    robot_description = rendered.stdout

    actions = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            parameters=[
                {
                    "config_file": str(
                        simulator_share / "config" / "ros_gz_bridge.yaml"
                    ),
                    "use_sim_time": True,
                    "expand_gz_topic_names": False,
                }
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            output="screen",
        ),
    ]

    gui = _enabled(values["gui"])
    paused = _enabled(values["paused"])
    run_args = [] if paused else ["-r"]
    base_gz_args = ["-v", "2", *run_args, str(world_path)]
    if gui:
        gz_launch = (
            Path(get_package_share_directory("ros_gz_sim"))
            / "launch"
            / "gz_sim.launch.py"
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_launch)),
                launch_arguments={
                    "gz_args": " ".join(shlex.quote(part) for part in base_gz_args),
                    "on_exit_shutdown": "true",
                }.items(),
            )
        )
    elif _enabled(values["headless_rendering"]):
        xvfb_runner = (
            Path(get_package_prefix("heron_simulator"))
            / "lib"
            / "heron_simulator"
            / "gazebo_with_xvfb.py"
        )
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(xvfb_runner),
                    "gz",
                    "sim",
                    "-s",
                    *base_gz_args,
                ],
                name="gazebo",
                output="screen",
                additional_env={
                    "GZ_SIM_RESOURCE_PATH": environment["GZ_SIM_RESOURCE_PATH"],
                    "GZ_SIM_SYSTEM_PLUGIN_PATH": environment[
                        "GZ_SIM_SYSTEM_PLUGIN_PATH"
                    ],
                },
                on_exit=Shutdown(reason="Gazebo Harmonic exited"),
            )
        )
    else:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        Path(get_package_share_directory("ros_gz_sim"))
                        / "launch"
                        / "gz_sim.launch.py"
                    )
                ),
                launch_arguments={
                    "gz_args": " ".join(
                        shlex.quote(part) for part in ["-s", *base_gz_args]
                    ),
                    "on_exit_shutdown": "true",
                }.items(),
            )
        )

    if _enabled(values["propulsion_adapter_enabled"]):
        actions.append(
            Node(
                package="heron_simulator",
                executable="drive_to_thrusters.py",
                name="cmd_drive_to_thrusters",
                parameters=[
                    str(dynamics_config_path),
                    {
                        "use_sim_time": True,
                        "namespace": namespace,
                        "drive_topic": values["drive_topic"],
                        "left_thruster_topic": values["left_thruster_topic"],
                        "right_thruster_topic": values["right_thruster_topic"],
                    },
                ],
                output="screen",
            )
        )

    if _enabled(values["simulate_multibeam_raw"]):
        actions.append(
            Node(
                package="heron_simulator",
                executable="multibeam_raw.py",
                name="multibeam_raw",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": values["sonar_ray_topic"],
                        "raw_topic": values["sonar_raw_topic"],
                        "frame_id": "dt100_link",
                        "extrinsic_revision": values["dt100_extrinsic_revision"],
                        "min_range_m": float(values["sonar_min_range_m"]),
                        "max_range_m": float(values["sonar_max_range_m"]),
                        "beam_count": 480,
                        "sector_size_deg": 120.0,
                        "start_angle_deg": -60.0,
                        "angle_increment_deg": 0.25,
                        "range_resolution_mm": int(
                            round(float(values["sonar_max_range_m"]) * 0.2)
                        ),
                        "acoustic_frequency_khz": 240,
                        "sound_speed_m_s": 1500.0,
                    }
                ],
                output="screen",
            )
        )
    if _enabled(values["simulate_ping360"]):
        actions.append(
            Node(
                package="heron_simulator",
                executable="ping360_profile_sim.py",
                name="ping360_profile_sim",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": values["ping360_ray_topic"],
                        "profile_topic": values["sonar_profile_topic"],
                        "frame_id": "ping360_link",
                        "extrinsic_revision": values["ping360_extrinsic_revision"],
                        "min_range_m": float(values["sonar_min_range_m"]),
                        "max_range_m": float(values["sonar_max_range_m"]),
                    }
                ],
                output="screen",
            )
        )
    if _enabled(values["simulated_housekeeping_enabled"]):
        actions.append(
            Node(
                package="heron_simulator",
                executable="sim_sense.py",
                name="sim_sense",
                parameters=[
                    {
                        "use_sim_time": True,
                        "topic": values["sense_topic"],
                        "rate_hz": float(values["sense_rate_hz"]),
                        "battery_v": float(values["sense_battery_v"]),
                        "vehicle_battery_topic": values["vehicle_battery_topic"],
                        "payload_battery_topic": values["payload_battery_topic"],
                        "status_topic": values["status_topic"],
                        "status_rate_hz": float(values["status_rate_hz"]),
                    }
                ],
                output="screen",
            )
        )
    if _enabled(values["use_ig_timing"]):
        actions.append(
            Node(
                package="heron_simulator",
                executable="sim_ig_timing.py",
                name="sim_ig_timing",
                parameters=[
                    {
                        "use_sim_time": True,
                        "pps_time_topic": values["pps_time_topic"],
                        "camera_time_topic": values["camera_time_topic"],
                        "imu_time_topic": values["imu_time_topic"],
                        "imu_topic": values["imu_topic"],
                        "pps_rate_hz": float(values["pps_rate_hz"]),
                        "camera_image_topics": values["camera_time_image_topics"],
                    }
                ],
                output="screen",
            )
        )

    if _enabled(values["use_joint_state_fallback"]):
        actions.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                namespace=namespace,
                parameters=[{"use_sim_time": True, "rate": 15.0}],
                output="screen",
            )
        )

    spawn_topic = (
        f"/{namespace}/robot_description" if namespace else "/robot_description"
    )
    create_arguments = [
        "-world",
        values["world_name"],
        "-name",
        model_name,
        "-topic",
        spawn_topic,
        "-x",
        values["x"],
        "-y",
        values["y"],
        "-z",
        values["z"],
        "-R",
        values["roll"],
        "-P",
        values["pitch"],
        "-Y",
        values["yaw"],
    ]
    actions.append(
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="heron_spawner",
                    arguments=create_arguments,
                    output="screen",
                )
            ],
        )
    )
    return actions


_ARGUMENTS = {
    "config": "ig_handle_benchmark",
    "sensor_frames_file": "",
    "world": "open_water.world",
    "world_name": "",
    "gui": "true",
    "headless_rendering": "true",
    "paused": "false",
    "namespace": "",
    "x": "0.0",
    "y": "0.0",
    "z": "0.0",
    "roll": "0.0",
    "pitch": "0.0",
    "yaw": "0.0",
    "drive_topic": "/cmd_drive",
    "left_thruster_topic": "/thrusters/left/input",
    "right_thruster_topic": "/thrusters/right/input",
    "thruster_dynamics_config": "",
    "propulsion_adapter_enabled": "true",
    "simulated_housekeeping_enabled": "true",
    "use_ig_timing": "true",
    "use_joint_state_fallback": "false",
    "simulate_multibeam_raw": "true",
    "simulate_ping360": "true",
    "sonar_ray_topic": "/sim/sensors/sonar/echosounder/rays",
    "sonar_raw_topic": "/sensors/sonar/echosounder/raw",
    "ping360_ray_topic": "/sim/sensors/sonar/imaging/rays",
    "sonar_profile_topic": "/sensors/sonar/imaging/profile",
    "dt100_extrinsic_revision": "dt100-seed-2026-08-11-v1",
    "ping360_extrinsic_revision": "ping360-seed-2026-08-11-v1",
    "sonar_min_range_m": "0.5",
    "sonar_max_range_m": "100.0",
    "sense_topic": "/sense",
    "sense_rate_hz": "10.0",
    "sense_battery_v": "16.0",
    "vehicle_battery_topic": "/battery/heron_state",
    "payload_battery_topic": "/sense_ighandle",
    "status_topic": "/status",
    "status_rate_hz": "1.0",
    "pps_time_topic": "/sensors/pps/time",
    "camera_time_topic": "/sensors/camera/time",
    "imu_time_topic": "/sensors/imu/time",
    "imu_topic": "/sensors/imu/data",
    "pps_rate_hz": "1.0",
    "camera_time_image_topics": "/sensors/camera/f1/image_raw,/sensors/camera/f2/image_raw,/sensors/camera/f3/image_raw,/sensors/camera/f4/image_raw",
    "disabled_sensor_ids": "",
    "lidar_update_rate_hz": "10",
    "lidar_horizontal_samples": "1800",
    "lidar_vertical_samples": "16",
    "hydrodynamics_override_enabled": "false",
    "base_mass_kg": "28.0",
    "fluid_density_kg_m3": "997.7735",
    "linear_damping_x": "-25.0",
    "quadratic_damping_x": "-5.0",
}


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in _ARGUMENTS.items()
    ]
    return LaunchDescription([*declarations, OpaqueFunction(function=_launch)])
