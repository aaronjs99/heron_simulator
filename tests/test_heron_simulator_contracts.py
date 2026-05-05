from __future__ import annotations

import importlib.util
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def load_module(name: str, relpath: str):
    path = REPO_ROOT / relpath
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def test_sim_worlds_and_mock_inspection_backend_exist():
    assert (REPO_ROOT / "heron_simulator/worlds/ocean_surface.world").exists()
    assert (REPO_ROOT / "heron_simulator/worlds/fathomwerx_pool.world").exists()
    assert (
        REPO_ROOT / "heron_simulator/scripts/autonomy/mock_defector_service.py"
    ).exists()


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
