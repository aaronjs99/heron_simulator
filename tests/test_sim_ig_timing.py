import importlib.util
import sys
import types
from pathlib import Path


class FakeTime:
    def __init__(self, secs=0, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs

    def to_nsec(self):
        return self.secs * 1_000_000_000 + self.nsecs

    @staticmethod
    def now():
        return FakeTime(42, 7)


def _load_module():
    rospy = types.ModuleType("rospy")
    rospy.Time = FakeTime
    sys.modules["rospy"] = rospy

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = type("Image", (), {})
    sensor_msgs_msg.Imu = type("Imu", (), {})
    sensor_msgs_msg.TimeReference = type("TimeReference", (), {})
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    path = Path(__file__).resolve().parents[1] / "scripts" / "sim_ig_timing.py"
    spec = importlib.util.spec_from_file_location("sim_ig_timing_test_module", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_topic_list_accepts_csv_semicolon_and_yaml_lists():
    module = _load_module()

    assert module.topic_list("/a,/b; /c") == ["/a", "/b", "/c"]
    assert module.topic_list(["/a", " /b ", ""]) == ["/a", "/b"]
    assert module.topic_list(None) == []


def test_valid_stamp_or_now_preserves_nonzero_and_replaces_zero():
    module = _load_module()
    stamp = FakeTime(3, 4)

    assert module.valid_stamp_or_now(stamp) is stamp
    assert module.valid_stamp_or_now(FakeTime()).to_nsec() == FakeTime.now().to_nsec()
