import math

import pytest

from omega7_fresh.dhd import DhdDevice


class FakeDhdLibrary:
    def dhdGetPositionAndOrientationFrame(self, px, py, pz, frame, _device_id):
        px._obj.value = 0.1
        py._obj.value = 0.2
        pz._obj.value = 0.3
        # C `double matrix[3][3]` is row-major and describes an active Rz(90°).
        for index, value in enumerate((0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)):
            frame[index] = value
        return 0

    def dhdGetGripperGap(self, gap, _device_id):
        gap._obj.value = 0.02
        return 0

    def dhdGetButton(self, _button, _device_id):
        return 0

    def dhdSetForceAndTorqueAndGripperForce(self, *_args):
        return 0


def test_dhd_position_order_and_row_major_orientation_frame():
    device = DhdDevice.__new__(DhdDevice)
    device._library = FakeDhdLibrary()
    device._device_id = 0

    sample = device.sample()

    assert sample.position == pytest.approx((0.1, 0.2, 0.3))
    assert sample.quaternion == pytest.approx(
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    )
    assert sample.gripper_gap == pytest.approx(0.02)
    assert sample.enabled is False
