from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_sim_worlds_and_mock_inspection_backend_exist():
    assert (REPO_ROOT / "heron_simulator/worlds/ocean_surface.world").exists()
    assert (REPO_ROOT / "heron_simulator/worlds/fathomwerx_pool.world").exists()
    assert (REPO_ROOT / "heron_simulator/scripts/autonomy/mock_defector_service.py").exists()
