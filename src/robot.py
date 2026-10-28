"""MIDI Mobile Manipulator Base Coordinator.

Integrates Gamepad input, Differential Drive kinematics, dual DDSM RS485 motors,
safety watchdog, and battery telemetry into an active control loop.
"""

from __future__ import annotations

import logging
import signal
import sys
import time
from typing import Dict, Any, Optional

from src.ddsm import DDSMDriver, MotorFeedback
from src.diff_drive import DifferentialDrive, RobotDimensions
from src.gamepad import F710Gamepad, GamepadState
from src.ups_battery import WaveshareUPSMonitor, BatteryTelemetry

logger = logging.getLogger("midi.robot")


class MidiRobot:
    """Mobile Manipulator Base Controller."""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.running = False

        # 1. Hardware Motor Configuration
        motors_cfg = config.get("motors", {})
        serial_cfg = config.get("serial", {})
        self.left_id = int(motors_cfg.get("left_id", 1))
        self.right_id = int(motors_cfg.get("right_id", 2))
        self.accel_param = int(motors_cfg.get("acceleration_param", 50))
        self.motor_type = motors_cfg.get("type", "ddsm115")

        self.driver = DDSMDriver(
            port=serial_cfg.get("port", "/dev/ttyUSB0"),
            baudrate=int(serial_cfg.get("baudrate", 115200)),
            timeout=float(serial_cfg.get("timeout", 0.05)),
            dry_run=bool(config.get("dry_run", False)),
            motor_type=self.motor_type,
        )

        # 2. Kinematics Configuration
        robot_cfg = config.get("robot", {})
        dims = RobotDimensions(
            wheel_radius=float(robot_cfg.get("wheel_radius", 0.0575)),
            wheel_base=float(robot_cfg.get("wheel_base", 0.350)),
            max_linear_speed=float(robot_cfg.get("max_linear_speed", 0.8)),
            max_angular_speed=float(robot_cfg.get("max_angular_speed", 2.0)),
            max_motor_rpm=float(robot_cfg.get("max_motor_rpm", 140.0)),
            invert_left=bool(motors_cfg.get("invert_left", False)),
            invert_right=bool(motors_cfg.get("invert_right", True)),
        )
        self.diff_drive = DifferentialDrive(
            dimensions=dims,
            linear_accel_limit=float(robot_cfg.get("linear_accel_limit", 1.5)),
            angular_accel_limit=float(robot_cfg.get("angular_accel_limit", 3.0)),
        )

        # 3. Gamepad Configuration
        gp_cfg = config.get("gamepad", {})
        self.gamepad = F710Gamepad(
            device_path=gp_cfg.get("device_path", None),
            deadzone=float(gp_cfg.get("deadzone", 0.10)),
            expo=float(gp_cfg.get("expo", 1.4)),
            require_deadman=bool(gp_cfg.get("require_deadman", False)),
            speed_scale_crawl=float(gp_cfg.get("speed_scale_crawl", 0.25)),
            speed_scale_normal=float(gp_cfg.get("speed_scale_normal", 0.65)),
            speed_scale_turbo=float(gp_cfg.get("speed_scale_turbo", 1.0)),
        )

        # 4. Battery / UPS HAT Configuration
        ups_cfg = config.get("ups_hat", {})
        self.ups = WaveshareUPSMonitor(
            i2c_bus=int(ups_cfg.get("i2c_bus", 1)),
            i2c_address=int(str(ups_cfg.get("i2c_address", "0x41")), 16),
            battery_type=ups_cfg.get("battery_type", "ups_2s"),
            enabled=bool(ups_cfg.get("enabled", True)),
        )

        # 5. Timing and Watchdog
        control_cfg = config.get("control", {})
        self.loop_hz = float(control_cfg.get("loop_hz", 25.0))
        self.watchdog_timeout = float(control_cfg.get("watchdog_timeout", 0.5))
        self.telemetry_interval = float(control_cfg.get("telemetry_interval", 1.0))

        # Internal state
        self.last_cmd_time = 0.0
        self.last_telemetry_time = 0.0
        self._target_left_rpm = 0.0
        self._target_right_rpm = 0.0

    def start(self):
        """Initialize all subsystems and run control loop."""
        logger.info("Initializing MIDI Mobile Manipulator Controller...")

        # Setup OS signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        # Connect RS485 serial bus
        if not self.driver.connect():
            logger.error("Could not initialize RS485 communication. Exiting.")
            sys.exit(1)

        # Set motors to Velocity Loop Mode
        logger.info("Setting DDSM motors (ID %d, ID %d) to Velocity Mode...", self.left_id, self.right_id)
        self.driver.set_mode(self.left_id, DDSMDriver.MODE_VELOCITY)
        time.sleep(0.02)
        self.driver.set_mode(self.right_id, DDSMDriver.MODE_VELOCITY)
        time.sleep(0.02)

        # Start Gamepad reader thread
        self.gamepad.start()

        # Initialize UPS HAT
        self.ups.init()

        logger.info("System ready. Control loop active at %.1f Hz. Waiting for gamepad...", self.loop_hz)
        self.running = True
        self._run_loop()

    def _run_loop(self):
        """High-frequency control loop."""
        dt = 1.0 / self.loop_hz

        while self.running:
            start_time = time.time()

            # 1. Read Gamepad inputs
            linear_norm, angular_norm, is_active = self.gamepad.get_drive_command()
            gp_state = self.gamepad.get_state()

            # 2. Check Watchdog safety
            time_since_event = time.time() - gp_state.last_event_time
            gamepad_timed_out = (gp_state.connected and time_since_event > self.watchdog_timeout)

            if not is_active or gamepad_timed_out:
                # Safe Stop
                target_v = 0.0
                target_w = 0.0
                is_braking = gp_state.emergency_stop_latched
            else:
                target_v = linear_norm * self.diff_drive.dims.max_linear_speed
                target_w = angular_norm * self.diff_drive.dims.max_angular_speed
                is_braking = False

            # 3. Kinematics calculation with acceleration slew rate limiting
            left_rpm, right_rpm = self.diff_drive.twist_to_rpm(
                target_v, target_w, apply_ramping=not is_braking
            )
            self._target_left_rpm = left_rpm
            self._target_right_rpm = right_rpm

            # 4. Transmit velocity commands to DDSM motors
            if is_braking:
                self.driver.emergency_brake(self.left_id)
                self.driver.emergency_brake(self.right_id)
            else:
                self.driver.set_velocity(
                    self.left_id, left_rpm, acceleration=self.accel_param
                )
                self.driver.set_velocity(
                    self.right_id, right_rpm, acceleration=self.accel_param
                )

            # 5. Periodic Telemetry and Logging
            now = time.time()
            if now - self.last_telemetry_time >= self.telemetry_interval:
                self.last_telemetry_time = now
                self._log_status(gp_state)

            # 6. Sleep remainder of loop interval
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _log_status(self, gp_state: GamepadState):
        """Log health and status periodically."""
        # Query feedback from motors
        fb_left = self.driver.query_feedback(self.left_id)
        fb_right = self.driver.query_feedback(self.right_id)
        batt = self.ups.read()

        l_act_rpm = fb_left.velocity_rpm if fb_left else 0.0
        r_act_rpm = fb_right.velocity_rpm if fb_right else 0.0
        est_v, est_w = self.diff_drive.rpm_to_twist(l_act_rpm, r_act_rpm)

        conn_str = "CONNECTED" if gp_state.connected else "WAITING"
        mode_str = f"[{gp_state.speed_mode.upper()}]"
        estop_str = " [E-STOP ACTIVE]" if gp_state.emergency_stop_latched else ""

        batt_info = ""
        if batt.connected:
            batt_info = f" | Batt: {batt.bus_voltage_v:.1f}V ({batt.percentage:.0f}%)"

        logger.info(
            "GP: %s %s%s | Cmd: L=%.1f R=%.1f RPM | Act: L=%.1f R=%.1f RPM (v=%.2fm/s w=%.2frad/s)%s",
            conn_str, mode_str, estop_str,
            self._target_left_rpm, self._target_right_rpm,
            l_act_rpm, r_act_rpm, est_v, est_w,
            batt_info,
        )

    def _handle_signal(self, signum, frame):
        """Safely handle termination signals."""
        logger.info("Signal %d received. Initiating safe robot shutdown...", signum)
        self.stop()
        sys.exit(0)

    def stop(self):
        """Halt motors, stop threads, close hardware connections."""
        self.running = False
        logger.info("Stopping robot...")

        # 1. Stop motors and apply brake
        try:
            self.diff_drive.emergency_stop()
            self.driver.emergency_brake(self.left_id)
            self.driver.emergency_brake(self.right_id)
            time.sleep(0.05)
            self.driver.set_velocity(self.left_id, 0.0, brake=True)
            self.driver.set_velocity(self.right_id, 0.0, brake=True)
        except Exception as e:
            logger.warning("Error commanding motor stop during shutdown: %s", e)

        # 2. Stop gamepad
        self.gamepad.stop()

        # 3. Close RS485 bus
        self.driver.close()
        logger.info("Robot shutdown complete.")
