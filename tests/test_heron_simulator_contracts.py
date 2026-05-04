from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


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
