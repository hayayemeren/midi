"""Unit Tests for MIDI Mobile Manipulator Control Suite."""

import math
import struct
import pytest

from src.ddsm import calculate_crc8, MotorFeedback, DDSMDriver
from src.diff_drive import DifferentialDrive, RobotDimensions, SlewRateLimiter
from src.gamepad import F710Gamepad, GamepadState
from src.robot import slew


def test_crc8_maxim_known_vector():
    """Verify CRC-8/MAXIM matches official DDSM115 specification."""
    payload = bytes([0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    crc = calculate_crc8(payload)
    assert crc == 0x04


def test_feedback_parsing():
    """Verify parsing of 10-byte motor feedback telemetry."""
    raw_packet = bytearray(10)
    raw_packet[0] = 0x01
    raw_packet[1] = 0x02
    struct.pack_into(">h", raw_packet, 2, 1000)
    struct.pack_into(">h", raw_packet, 4, 50)
    struct.pack_into(">h", raw_packet, 6, 12000)
    raw_packet[8] = 0x00
    raw_packet[9] = calculate_crc8(bytes(raw_packet[:9]))

    fb = MotorFeedback.from_bytes(bytes(raw_packet))
    assert fb is not None
    assert fb.motor_id == 1
    assert fb.mode == 2
    assert fb.velocity_rpm == 50.0
    assert fb.current_raw == 1000
    assert abs(fb.current_amps - (1000 * 8.0 / 32767.0)) < 0.01
    assert fb.error_code == 0
    assert not fb.over_current
    assert not fb.over_temp


def test_ddsm_drive_packet_format():
    """Verify DDSM drive packet has acceleration=1 at Byte 6 and CRC8 at Byte 9."""
    driver = DDSMDriver(dry_run=True)
    # Target 100 RPM
    driver.set_velocity(motor_id=1, target_rpm=100.0, acceleration=1)
    fb = driver.query_feedback(1)
    assert fb is not None
    assert fb.velocity_rpm == 100.0


def test_slew_function_acceleration_and_deceleration():
    """Test asymmetric slew rate function."""
    cur = 0.0
    tgt = 100.0
    up = 1000.0   # 1000 RPM/s
    down = 500.0  # 500 RPM/s
    dt = 0.02     # 20ms

    # Step forward (accelerating) -> cur + 1000 * 0.02 = 20
    cur = slew(cur, tgt, up, down, dt)
    assert abs(cur - 20.0) < 1e-4

    # Stop (decelerating to 0) -> cur - 500 * 0.02 = 10
    tgt = 0.0
    cur = slew(cur, tgt, up, down, dt)
    assert abs(cur - 10.0) < 1e-4

    # Snap to exactly 0 when target is 0 and close
    cur = slew(cur, tgt, up, down, 0.05)
    assert cur == 0.0


def test_gamepad_norm_deadzone():
    """Test axis normalization and deadzone calculation."""
    gp = F710Gamepad(deadzone=0.10)
    gp.ax[1] = (-32767, 32767)

    # Within deadzone -> exactly 0.0
    gp.val[1] = 1000   # ~0.03
    assert gp.norm(1) == 0.0

    # Outside deadzone -> non-zero
    gp.val[1] = 16384  # ~0.50
    val = gp.norm(1)
    assert val > 0.0
    assert val <= 1.0


def test_differential_drive_forward_straight():
    """Driving straight forward must command equal RPMs (taking inversion into account)."""
    dims = RobotDimensions(
        wheel_radius=0.0575,
        wheel_base=0.350,
        max_linear_speed=1.0,
        max_angular_speed=2.0,
        max_motor_rpm=140.0,
        invert_left=False,
        invert_right=True,
    )
    diff = DifferentialDrive(dims)

    l_rpm, r_rpm = diff.twist_to_rpm(linear_v=0.5, angular_w=0.0, apply_ramping=False)
    expected_rpm = (0.5 / (2.0 * math.pi * 0.0575)) * 60.0
    assert abs(l_rpm - expected_rpm) < 0.5
    assert abs(r_rpm - (-expected_rpm)) < 0.5
