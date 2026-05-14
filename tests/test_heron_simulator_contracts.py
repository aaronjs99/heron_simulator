from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]


def load_module(name: str, relpath: str):
    path = REPO_ROOT / relpath
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def test_sim_worlds_and_autonomy_runtime_assets_exist():
    assert (REPO_ROOT / "heron_simulator/worlds/ocean_surface.world").exists()
    assert (REPO_ROOT / "heron_simulator/worlds/fathomwerx_pool.world").exists()
    assert (
        REPO_ROOT / "heron_simulator/scripts/autonomy/spawn_inspection_models.py"
    ).exists()
    cmake = (REPO_ROOT / "heron_simulator/CMakeLists.txt").read_text(encoding="utf-8")
    assert "spawn_inspection_models.py" in cmake


def test_gui_launch_sites_use_shared_window_layout_helper():
    for relpath in (
        "heron_simulator/launch/heron_world.launch",
        "heron_simulator/launch/gazebo_world.launch",
        "heron_simulator/launch/heron_lake_world.launch",
        "heron_simulator/launch/local_playback.launch",
        "heron_simulator/launch/dlio.launch",
    ):
        text = (REPO_ROOT / relpath).read_text(encoding="utf-8")
        assert "gui_window_layout.launch" in text or relpath.endswith("dlio.launch")
        assert "arrange_gui_windows" in text
        assert "demo_window_layout" in text


def test_playback_and_lake_world_do_not_reference_missing_assets():
    playback = (REPO_ROOT / "heron_simulator/launch/local_playback.launch").read_text(
        encoding="utf-8"
    )
    lake = (REPO_ROOT / "heron_simulator/worlds/lake.world").read_text(encoding="utf-8")
    package_xml = (REPO_ROOT / "heron_simulator/package.xml").read_text(
        encoding="utf-8"
    )

    assert "$(find slam_grande)/config/rviz/nav_debug.rviz" in playback
    assert "config/playback.rviz" not in playback
    assert "model://ned_frame" not in lake
    assert "<exec_depend>defector</exec_depend>" not in package_xml


def test_sim_imu_body_and_frame_match_dlio_geometry():
    extras = (
        REPO_ROOT / "heron_simulator/urdf/ig_handle_sensors.urdf.xacro"
    ).read_text(encoding="utf-8")

    assert "<bodyName>imu_link</bodyName>" in extras
    assert "<frameId>imu_link</frameId>" in extras


def test_sim_sonar_matches_dt100_profile_geometry():
    for relpath in (
        "heron_simulator/urdf/ig_handle_sensors.urdf.xacro",
        "heron_simulator/urdf/heron_sim.urdf.xacro",
    ):
        text = (REPO_ROOT / relpath).read_text(encoding="utf-8")
        sonar_block = text[text.index('<sensor type="ray" name="sonar">') :]
        sonar_block = sonar_block[: sonar_block.index("</sensor>")]

        assert '<origin xyz="0 0 -0.08" rpy="0 1.5708 0"/>' in text
        assert "<update_rate>20</update_rate>" in sonar_block
        assert "<samples>480</samples>" in sonar_block
        assert "<min_angle>-1.047198</min_angle>" in sonar_block
        assert "<max_angle>1.047198</max_angle>" in sonar_block
        assert "<samples>1</samples>" in sonar_block
        assert "<min>0.5</min>" in sonar_block
        assert "<max>100.0</max>" in sonar_block
        assert "<min_range>0.5</min_range>" in sonar_block
        assert "<max_range>100.0</max_range>" in sonar_block
        assert "<topicName>/sensors/sonar/scan</topicName>" in sonar_block
        assert "<frameName>sonar_link</frameName>" in sonar_block


def test_sim_lidars_match_vlp16_geometry_contract():
    for relpath in (
        "heron_simulator/urdf/ig_handle_sensors.urdf.xacro",
        "heron_simulator/urdf/heron_sim.urdf.xacro",
    ):
        text = (REPO_ROOT / relpath).read_text(encoding="utf-8")
        for name, topic, frame in (
            ("lidar_h_sensor", "/sensors/lidar/hori/points", "lidar_h_link"),
            ("lidar_v_sensor", "/sensors/lidar/vert/points", "lidar_v_link"),
        ):
            block = text[text.index(f'<sensor type="ray" name="{name}">') :]
            block = block[: block.index("</sensor>")]

            assert "<update_rate>10</update_rate>" in block
            assert "<samples>1800</samples>" in block
            assert "<samples>16</samples>" in block
            assert "<min_angle>-3.14159</min_angle>" in block
            assert "<max_angle>3.14159</max_angle>" in block
            assert "<min_angle>-0.261799</min_angle>" in block
            assert "<max_angle>0.261799</max_angle>" in block
            assert "<max>200.0</max>" in block
            assert "<max_range>200.0</max_range>" in block
            assert f"<topicName>{topic}</topicName>" in block
            assert f"<frameName>{frame}</frameName>" in block


def test_sim_cameras_match_forge_profile_contract():
    for relpath in (
        "heron_simulator/urdf/ig_handle_sensors.urdf.xacro",
        "heron_simulator/urdf/heron_sim.urdf.xacro",
    ):
        text = (REPO_ROOT / relpath).read_text(encoding="utf-8")
        for name, frame in (
            ("camera_F1", "camera_F1_optical_link"),
            ("camera_F2", "camera_F2_optical_link"),
        ):
            sensor_name = f"{name}_sensor" if "ig_handle_sensors" in relpath else name
            block = text[text.index(f'<sensor type="camera" name="{sensor_name}">') :]
            block = block[: block.index("</sensor>")]

            assert "<update_rate>15.0</update_rate>" in block
            assert "<horizontal_fov>1.60</horizontal_fov>" in block
            assert "<width>1280</width>" in block
            assert "<height>1024</height>" in block
            assert "<updateRate>15.0</updateRate>" in block
            assert f"<frameName>{frame}</frameName>" in block


def test_sim_launches_use_portable_xacro_executable():
    for relpath in (
        "heron_simulator/launch/spawn_heron.launch",
        "heron_simulator/launch/local_playback.launch",
    ):
        text = (REPO_ROOT / relpath).read_text(encoding="utf-8")
        assert "$(find xacro)/xacro" not in text
        assert "xacro '$(find heron_description)/urdf/heron.urdf.xacro'" in text


def test_heron_spawn_is_one_shot_without_shutdown_delete_bond():
    text = (REPO_ROOT / "heron_simulator/launch/spawn_heron.launch").read_text(
        encoding="utf-8"
    )

    assert 'type="spawn_model"' in text
    assert "-param robot_description" in text
    assert " -b" not in text
    assert 'required="true"' not in text


def test_sim_preflight_cleanup_is_explicit_opt_in():
    text = (REPO_ROOT / "heron_simulator/scripts/tools/sim_preflight.py").read_text(
        encoding="utf-8"
    )

    assert 'rospy.get_param("~auto_cleanup", False)' in text
    assert 'rospy.get_param("~cleanup_stale_ros_nodes", False)' in text
    assert "if cleanup_stale_ros_nodes:" in text


def test_cmd_drive_translate_maps_negative_drive_to_reverse_thrust():
    translator_mod = load_module(
        "cmd_drive_translate_contract",
        "heron_simulator/scripts/control/cmd_drive_translate.py",
    )
    translator = translator_mod.ThrusterTranslator.__new__(
        translator_mod.ThrusterTranslator
    )
    translator.max_fwd_thrust = 45.0
    translator.max_bck_thrust = 25.0

    assert translator.drive_to_thrust(-0.40) == -10.0
    assert translator.drive_to_thrust(0.40) == 18.0


def test_sim_control_profiles_only_expose_active_node_parameters():
    twist_keys = {
        "max_linear",
        "max_angular",
        "yaw_mix",
        "shared_drive_normalization",
        "rate",
        "timeout",
    }
    thrust_keys = {
        "rate",
        "cmd_timeout",
        "max_fwd_thrust",
        "max_bck_thrust",
        "left_scale",
        "right_scale",
    }

    for relpath in ("heron_simulator/config/twist_to_drive.yaml",):
        cfg = yaml.safe_load((REPO_ROOT / relpath).read_text(encoding="utf-8"))
        assert set(cfg) == twist_keys

    for relpath in ("heron_simulator/config/thruster_dynamics.yaml",):
        cfg = yaml.safe_load((REPO_ROOT / relpath).read_text(encoding="utf-8"))
        assert set(cfg) == thrust_keys
