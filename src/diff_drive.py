"""Differential Drive Kinematics & Velocity Profiling for Mobile Manipulator.

Converts linear velocity (v) and angular velocity (omega) into target RPMs
for left and right DDSM hub motors, with slew-rate (acceleration) limiting.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Tuple


@dataclass
class RobotDimensions:
    """Mobile base physical geometric parameters."""
    wheel_radius: float = 0.0575   # meters (DDSM115 is ~115mm diameter -> radius 0.0575m)
    wheel_base: float = 0.350      # meters (distance between left and right contact points)
    max_linear_speed: float = 0.8  # m/s (~2.88 km/h safe maneuvering speed)
    max_angular_speed: float = 2.0 # rad/s (~115 deg/s)
    max_motor_rpm: float = 140.0   # DDSM115 rated speed limit
    invert_left: bool = False      # Invert rotation direction if wheel is mounted mirrored
    invert_right: bool = True      # Typically right wheel is mounted opposite to left


class SlewRateLimiter:
    """Smooths abrupt setpoint jumps to avoid tipping and high current surges."""

    def __init__(self, max_accel: float, initial_value: float = 0.0):
        """Args:
            max_accel: Maximum change allowed per second (units/s^2).
            initial_value: Starting value.
        """
        self.max_accel = max_accel
        self.current_value = initial_value
        self.last_update = time.time()

    def update(self, target: float) -> float:
        now = time.time()
        # Clamp dt to prevent massive jumps after delays or pauses
        dt = max(0.001, min(0.1, now - self.last_update))
        self.last_update = now

        max_delta = self.max_accel * dt
        delta = target - self.current_value

        # Snap immediately to zero when releasing stick near deadband
        if abs(target) < 1e-4 and abs(self.current_value) < max(0.05, max_delta):
            self.current_value = 0.0
            return 0.0

        if abs(delta) <= max_delta:
            self.current_value = target
        else:
            self.current_value += math.copysign(max_delta, delta)

        return self.current_value

    def reset(self, value: float = 0.0):
        self.current_value = value
        self.last_update = time.time()


class DifferentialDrive:
    """Converts twist commands (v, omega) to left/right motor RPMs."""

    def __init__(
        self,
        dimensions: RobotDimensions,
        linear_accel_limit: float = 1.5,   # m/s^2
        angular_accel_limit: float = 3.0,  # rad/s^2
    ):
        self.dims = dimensions
        self.linear_limiter = SlewRateLimiter(linear_accel_limit)
        self.angular_limiter = SlewRateLimiter(angular_accel_limit)

    def twist_to_rpm(
        self,
        linear_v: float,
        angular_w: float,
        apply_ramping: bool = True,
    ) -> Tuple[float, float]:
        """Convert linear (m/s) and angular (rad/s) velocities to (left_rpm, right_rpm).
        
        Args:
            linear_v: Forward/backward speed in m/s.
            angular_w: Yaw rotation speed in rad/s (positive = turn left / CCW).
            apply_ramping: If True, filters commands through acceleration limiters.
            
        Returns:
            Tuple of (left_motor_rpm, right_motor_rpm).
        """
        # Clamp inputs to robot limits
        linear_v = max(-self.dims.max_linear_speed, min(self.dims.max_linear_speed, linear_v))
        angular_w = max(-self.dims.max_angular_speed, min(self.dims.max_angular_speed, angular_w))

        if apply_ramping:
            v = self.linear_limiter.update(linear_v)
            w = self.angular_limiter.update(angular_w)
        else:
            v = linear_v
            w = angular_w
            self.linear_limiter.reset(v)
            self.angular_limiter.reset(w)

        # Standard 2-wheel differential drive kinematics:
        # v_left  = v - (omega * L) / 2
        # v_right = v + (omega * L) / 2
        half_base = self.dims.wheel_base / 2.0
        v_left = v - (w * half_base)
        v_right = v + (w * half_base)

        # Convert linear wheel speed (m/s) to RPM:
        # RPM = (v / (2 * pi * R)) * 60
        left_rpm = (v_left / (2.0 * math.pi * self.dims.wheel_radius)) * 60.0
        right_rpm = (v_right / (2.0 * math.pi * self.dims.wheel_radius)) * 60.0

        # Scale down both wheels proportionally if either exceeds max_motor_rpm
        peak_rpm = max(abs(left_rpm), abs(right_rpm))
        if peak_rpm > self.dims.max_motor_rpm and peak_rpm > 0.0:
            scale = self.dims.max_motor_rpm / peak_rpm
            left_rpm *= scale
            right_rpm *= scale

        # Apply motor mounting direction inversion
        if self.dims.invert_left:
            left_rpm = -left_rpm
        if self.dims.invert_right:
            right_rpm = -right_rpm

        return round(left_rpm, 2), round(right_rpm, 2)

    def rpm_to_twist(self, left_rpm: float, right_rpm: float) -> Tuple[float, float]:
        """Convert actual motor feedback RPMs back to estimated robot twist (v, omega)."""
        # Undo inversions
        l_rpm = -left_rpm if self.dims.invert_left else left_rpm
        r_rpm = -right_rpm if self.dims.invert_right else right_rpm

        # RPM to linear speed
        v_left = (l_rpm * 2.0 * math.pi * self.dims.wheel_radius) / 60.0
        v_right = (r_rpm * 2.0 * math.pi * self.dims.wheel_radius) / 60.0

        # Kinematics:
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.dims.wheel_base

        return round(v, 3), round(w, 3)

    def emergency_stop(self):
        """Immediately reset all internal acceleration states to 0."""
        self.linear_limiter.reset(0.0)
        self.angular_limiter.reset(0.0)
