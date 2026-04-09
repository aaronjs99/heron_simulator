#!/usr/bin/env python3
import importlib.util
import os
import unittest


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "autonomy", "spawn_inspection_models.py")


def _load_module():
    spec = importlib.util.spec_from_file_location("spawn_inspection_models", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _FakeRospy:
    class ServiceException(Exception):
        pass

    class ROSException(Exception):
        pass

    class ROSInterruptException(Exception):
        pass

    @staticmethod
    def is_shutdown():
        return True


class SpawnInspectionModelsShutdownTests(unittest.TestCase):
    def test_call_spawn_model_raises_interrupt_on_shutdown_without_type_error(self):
        module = _load_module()
        original_rospy = module.rospy
        module.rospy = _FakeRospy
        try:
            with self.assertRaises(_FakeRospy.ROSInterruptException):
                module._call_spawn_model(
                    proxy=object(),
                    anchor_id="dock_test",
                    sdf="<sdf/>",
                    pose=object(),
                    retries=2,
                )
        finally:
            module.rospy = original_rospy


if __name__ == "__main__":
    unittest.main()
