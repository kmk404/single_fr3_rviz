"""Tests for the small Force Dimension DHD binding."""

from omega7_teleop.dhd import DHD_OFF, DHD_ON, DhdDevice, DhdError
import pytest


class FakeDhdLibrary:
    """Record DHD calls while returning one valid Omega.7 sample."""

    def __init__(self):
        self.calls = []
        self.button = DHD_ON
        self.emulate_result = 0

    def dhdOpen(self):
        self.calls.append(("open",))
        return 0

    def dhdClose(self, _device_id):
        self.calls.append(("close",))
        return 0

    def dhdGetSystemType(self, _device_id):
        return 35

    def dhdGetSystemName(self, _device_id):
        return b"omega.7"

    def dhdHasWrist(self, _device_id):
        return True

    def dhdHasGripper(self, _device_id):
        return True

    def dhdEnableForce(self, value, _device_id):
        self.calls.append(("force", value))
        return 0

    def dhdEmulateButton(self, value, _device_id):
        self.calls.append(("emulate", value))
        return self.emulate_result if value == DHD_ON else 0

    def dhdSetGravityCompensation(self, value, _device_id):
        self.calls.append(("gravity", value))
        return 0

    def dhdSetForceAndTorqueAndGripperForce(self, *_arguments):
        self.calls.append(("zero_force",))
        return 0

    def dhdGetPositionAndOrientationFrame(
        self, px, py, pz, frame, _device_id
    ):
        px._obj.value = 0.01
        py._obj.value = -0.02
        pz._obj.value = 0.03
        for index, value in enumerate((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)):
            frame[index] = value
        return 0

    def dhdGetGripperGap(self, gap, _device_id):
        gap._obj.value = 0.005
        return 0

    def dhdGetButton(self, index, _device_id):
        assert index == 0
        return self.button

    def dhdErrorGetLastStr(self):
        return b"fake DHD failure"


def make_device(library):
    """Create a DhdDevice around the fake library without loading a shared object."""
    device = DhdDevice.__new__(DhdDevice)
    device._library = library
    device._device_id = None
    device._require_omega7 = True
    return device


def test_gripper_is_emulated_as_button_zero_and_disabled_on_close():
    library = FakeDhdLibrary()
    device = make_device(library)

    assert device.connect() == "omega.7"
    assert library.calls.index(("force", DHD_ON)) < library.calls.index(
        ("emulate", DHD_ON)
    )
    assert device.sample().enabled is True
    library.button = DHD_OFF
    assert device.sample().enabled is False

    device.close()
    assert library.calls[-3:] == [
        ("emulate", DHD_OFF),
        ("force", DHD_OFF),
        ("close",),
    ]
    assert device.connected is False


def test_connection_setup_failure_closes_device_and_disables_force():
    library = FakeDhdLibrary()
    library.emulate_result = -1
    device = make_device(library)

    with pytest.raises(DhdError, match="dhdEmulateButton"):
        device.connect()

    assert ("force", DHD_OFF) in library.calls
    assert library.calls[-1] == ("close",)
    assert device.connected is False
