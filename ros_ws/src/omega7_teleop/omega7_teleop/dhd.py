"""Minimal ctypes binding for the DHD calls used by the Omega.7 node."""

import ctypes
from dataclasses import dataclass

from .math3d import finite, matrix_to_quaternion


DHD_OFF = 0
DHD_ON = 1
DHD_DEVICE_OMEGA7_RIGHT = 35
DHD_DEVICE_OMEGA7_LEFT = 37


class DhdError(RuntimeError):
    """A Force Dimension SDK operation failed."""


@dataclass(frozen=True)
class Sample:
    position: tuple
    quaternion: tuple
    gripper_gap: float
    enabled: bool


class DhdDevice:
    """Own one Omega.7 connection and return coherent pose/gripper samples."""

    def __init__(self, library_path, require_omega7=True):
        try:
            self._library = ctypes.CDLL(library_path)
        except OSError as error:
            raise DhdError(f"cannot load {library_path}: {error}") from error
        self._configure_signatures()
        self._device_id = None
        self._require_omega7 = require_omega7

    def _configure_signatures(self):
        lib = self._library
        lib.dhdOpen.argtypes = []
        lib.dhdOpen.restype = ctypes.c_int
        lib.dhdClose.argtypes = [ctypes.c_byte]
        lib.dhdClose.restype = ctypes.c_int
        lib.dhdGetSystemType.argtypes = [ctypes.c_byte]
        lib.dhdGetSystemType.restype = ctypes.c_int
        lib.dhdGetSystemName.argtypes = [ctypes.c_byte]
        lib.dhdGetSystemName.restype = ctypes.c_char_p
        lib.dhdHasWrist.argtypes = [ctypes.c_byte]
        lib.dhdHasWrist.restype = ctypes.c_bool
        lib.dhdHasGripper.argtypes = [ctypes.c_byte]
        lib.dhdHasGripper.restype = ctypes.c_bool
        lib.dhdEmulateButton.argtypes = [ctypes.c_ubyte, ctypes.c_byte]
        lib.dhdEmulateButton.restype = ctypes.c_int
        lib.dhdEnableForce.argtypes = [ctypes.c_ubyte, ctypes.c_byte]
        lib.dhdEnableForce.restype = ctypes.c_int
        lib.dhdSetGravityCompensation.argtypes = [ctypes.c_int, ctypes.c_byte]
        lib.dhdSetGravityCompensation.restype = ctypes.c_int
        lib.dhdGetButton.argtypes = [ctypes.c_int, ctypes.c_byte]
        lib.dhdGetButton.restype = ctypes.c_int
        lib.dhdGetGripperGap.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_byte]
        lib.dhdGetGripperGap.restype = ctypes.c_int
        matrix_pointer = ctypes.POINTER(ctypes.c_double)
        lib.dhdGetPositionAndOrientationFrame.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            matrix_pointer,
            ctypes.c_byte,
        ]
        lib.dhdGetPositionAndOrientationFrame.restype = ctypes.c_int
        lib.dhdSetForceAndTorqueAndGripperForce.argtypes = [
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_byte,
        ]
        lib.dhdSetForceAndTorqueAndGripperForce.restype = ctypes.c_int
        lib.dhdErrorGetLastStr.argtypes = []
        lib.dhdErrorGetLastStr.restype = ctypes.c_char_p

    @property
    def connected(self):
        return self._device_id is not None

    def _id(self):
        if self._device_id is None:
            raise DhdError("Omega.7 is not connected")
        return ctypes.c_byte(self._device_id)

    def _last_error(self):
        value = self._library.dhdErrorGetLastStr()
        return value.decode("utf-8", errors="replace") if value else "unknown DHD error"

    def _check(self, result, operation):
        if result < 0:
            raise DhdError(f"{operation} failed: {self._last_error()}")

    def connect(self):
        if self.connected:
            return self.name
        device_id = self._library.dhdOpen()
        if device_id < 0:
            raise DhdError(f"dhdOpen failed: {self._last_error()}")
        self._device_id = device_id
        try:
            device_type = self._library.dhdGetSystemType(self._id())
            if self._require_omega7 and device_type not in (
                DHD_DEVICE_OMEGA7_RIGHT,
                DHD_DEVICE_OMEGA7_LEFT,
            ):
                raise DhdError(
                    f"expected Omega.7 (type 35/37), detected {self.name} "
                    f"(type {device_type})"
                )
            if not self._library.dhdHasWrist(self._id()):
                raise DhdError("detected device has no 3-DOF wrist")
            if not self._library.dhdHasGripper(self._id()):
                raise DhdError("detected device has no gripper input")
            # Force output must be enabled before the Omega.7 can provide gravity
            # compensation.  Its gripper is then exposed as virtual button 0.
            self._check(
                self._library.dhdEnableForce(DHD_ON, self._id()),
                "dhdEnableForce",
            )
            self._check(
                self._library.dhdEmulateButton(DHD_ON, self._id()),
                "dhdEmulateButton",
            )
            self._check(
                self._library.dhdSetGravityCompensation(DHD_ON, self._id()),
                "dhdSetGravityCompensation",
            )
            self.zero_force()
            return self.name
        except Exception:
            self.close()
            raise

    @property
    def name(self):
        value = self._library.dhdGetSystemName(self._id())
        return value.decode("utf-8", errors="replace") if value else "unknown"

    def zero_force(self):
        self._check(
            self._library.dhdSetForceAndTorqueAndGripperForce(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self._id()
            ),
            "dhdSetForceAndTorqueAndGripperForce",
        )

    def sample(self):
        px = ctypes.c_double()
        py = ctypes.c_double()
        pz = ctypes.c_double()
        frame = (ctypes.c_double * 9)()
        self._check(
            self._library.dhdGetPositionAndOrientationFrame(
                ctypes.byref(px),
                ctypes.byref(py),
                ctypes.byref(pz),
                frame,
                self._id(),
            ),
            "dhdGetPositionAndOrientationFrame",
        )
        gap = ctypes.c_double()
        self._check(
            self._library.dhdGetGripperGap(ctypes.byref(gap), self._id()),
            "dhdGetGripperGap",
        )
        button = self._library.dhdGetButton(0, self._id())
        self._check(button, "dhdGetButton(0)")
        self.zero_force()

        position = (px.value, py.value, pz.value)
        matrix = tuple(tuple(frame[row * 3 + column] for column in range(3)) for row in range(3))
        values = position + tuple(value for row in matrix for value in row) + (gap.value,)
        if not finite(values):
            raise DhdError("Omega.7 returned non-finite data")
        quaternion = matrix_to_quaternion(matrix)
        return Sample(position, quaternion, gap.value, button != DHD_OFF)

    def close(self):
        if not self.connected:
            return
        device_id = self._id()
        try:
            self._library.dhdSetForceAndTorqueAndGripperForce(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, device_id
            )
            self._library.dhdEmulateButton(DHD_OFF, device_id)
            self._library.dhdEnableForce(DHD_OFF, device_id)
        finally:
            self._library.dhdClose(device_id)
            self._device_id = None
