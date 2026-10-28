"""Unit Tests for MIDI Mobile Manipulator Control Suite."""

import math
import struct
import pytest

from src.ddsm import calculate_crc8, MotorFeedback, DDSMDriver
from src.diff_drive import DifferentialDrive, RobotDimensions, SlewRateLimiter
from src.gamepad import F710Gamepad, GamepadState


def test_crc8_maxim_known_vector():
    """Verify CRC-8/MAXIM matches official DDSM115 specification."""
    # Official feedback query packet: [0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] -> CRC is 0x04
    payload = bytes([0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    crc = calculate_crc8(payload)
    assert crc == 0x04


def test_feedback_parsing():
    """Verify parsing of 10-byte motor feedback telemetry."""
    # Construct a valid response packet for ID 1:
    # ID: 1, Mode: 2 (Velocity), Current: 1000 (raw), Speed: 50 (RPM), Pos: 12000, Err: 0
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


def test_feedback_crc_failure():
    """Corrupted packets must be rejected."""
    bad_packet = bytes([0x01, 0x02, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0xFF])
    assert MotorFeedback.from_bytes(bad_packet) is None


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

    # Move straight at 0.5 m/s, 0.0 rad/s
    l_rpm, r_rpm = diff.twist_to_rpm(linear_v=0.5, angular_w=0.0, apply_ramping=False)

    # Theoretical RPM = (0.5 / (2 * pi * 0.0575)) * 60 ~= 83.04 RPM
    expected_rpm = (0.5 / (2.0 * math.pi * 0.0575)) * 60.0

    assert abs(l_rpm - expected_rpm) < 0.5
    # Right wheel inverted
    assert abs(r_rpm - (-expected_rpm)) < 0.5


def test_differential_drive_spin_in_place():
    """Pure turn in place should command opposing wheel velocities."""
    dims = RobotDimensions(
        wheel_radius=0.0575,
        wheel_base=0.350,
        invert_left=False,
        invert_right=True,
    )
    diff = DifferentialDrive(dims)

    # Turn left (positive angular velocity = CCW)
    l_rpm, r_rpm = diff.twist_to_rpm(linear_v=0.0, angular_w=1.0, apply_ramping=False)

    # Left wheel should move backwards (-), Right wheel forwards (which inverted is -)
    assert l_rpm < 0
    # Both motors will turn in directions causing CCW rotation
    v_est, w_est = diff.rpm_to_twist(l_rpm, r_rpm)
    assert abs(v_est) < 0.02
    assert abs(w_est - 1.0) < 0.05


def test_slew_rate_limiter():
    """Slew rate limiter must constrain acceleration steps."""
    limiter = SlewRateLimiter(max_accel=1.0, initial_value=0.0)
    # Step jump to 10.0
    val = limiter.update(10.0)
    # Immediately after start, value should be constrained to small step
    assert val < 1.0


def test_gamepad_deadzone():
    """Sticks within deadzone must evaluate to 0.0."""
    gp = F710Gamepad(deadzone=0.10)
    assert gp._apply_deadzone_and_expo(0.05) == 0.0
    assert gp._apply_deadzone_and_expo(-0.08) == 0.0
    assert abs(gp._apply_deadzone_and_expo(0.5)) > 0.0


def test_gamepad_drive_command_safety():
    """If gamepad is disconnected, drive commands must always be 0."""
    gp = F710Gamepad()
    lin, ang, active = gp.get_drive_command()
    assert lin == 0.0
    assert ang == 0.0
    assert not active


def test_gamepad_stick_mapping_and_speed_bumpers():
    """Verify Left stick Y controls linear, Right stick X controls steer, LB/RB control speed."""
    gp = F710Gamepad(speed_levels=[0.20, 0.40, 0.60, 0.80, 1.00], default_speed_level=3)
    gp._state.connected = True

    # Default is Level 3 (0.60)
    assert gp._state.speed_level == 3
    assert abs(gp._state.speed_multiplier - 0.60) < 1e-4

    # 1. Test stick mapping: Left stick Y = 1.0 (Forward), Right stick X = 0.5 (Steer Right)
    gp._state.left_stick_y = 1.0
    gp._state.right_stick_x = 0.5
    gp._state.left_stick_x = 0.9  # Should be ignored for steering

    lin, ang, active = gp.get_drive_command()
    assert active
    assert abs(lin - 0.60) < 1e-4  # 1.0 * 0.60
    assert abs(ang - (-0.30)) < 1e-4  # -0.5 * 0.60 (right steer = negative ang)

    # 2. Test LB (Speed UP): Level 3 -> 4 -> 5
    gp.speed_up()
    assert gp._state.speed_level == 4
    assert abs(gp._state.speed_multiplier - 0.80) < 1e-4

    gp.speed_up()
    assert gp._state.speed_level == 5
    assert abs(gp._state.speed_multiplier - 1.00) < 1e-4

    # Speed up should cap at max (5)
    gp.speed_up()
    assert gp._state.speed_level == 5

    # 3. Test RB (Speed DOWN): Level 5 -> 4 -> 3 -> 2 -> 1
    gp.speed_down()
    assert gp._state.speed_level == 4
    gp.speed_down()
    assert gp._state.speed_level == 3
    gp.speed_down()
    assert gp._state.speed_level == 2
    gp.speed_down()
    assert gp._state.speed_level == 1
    assert abs(gp._state.speed_multiplier - 0.20) < 1e-4

    # Speed down should floor at min (1)
    gp.speed_down()
    assert gp._state.speed_level == 1


def test_continuous_driving_and_instant_stop():
    """Verify that holding stick steady produces continuous commands, and releasing immediately stops."""
    gp = F710Gamepad()
    gp._state.connected = True
    gp._state.left_stick_y = 1.0  # Hold stick forward

    # 1. While held, output should remain continuous and active
    for _ in range(10):
        lin, ang, active = gp.get_drive_command()
        assert active
        assert lin > 0.0

    # 2. Release stick to 0.0
    gp._state.left_stick_y = 0.0
    lin, ang, active = gp.get_drive_command()
    assert lin == 0.0
    assert ang == 0.0


