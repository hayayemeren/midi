"""MIDI Mobile Manipulator Base Controller.

Synchronous 50 Hz control loop with evdev gamepad polling, asymmetric slew rate
ramping, and collision-free RS485 DDSM motor drive.
"""

from __future__ import annotations

import logging
import signal
import sys
import time
from typing import Dict, Any, Optional

from src.ddsm import DDSMDriver, MotorFeedback
from src.gamepad import F710Gamepad, GamepadState
from src.ups_battery import WaveshareUPSMonitor, BatteryTelemetry

logger = logging.getLogger("midi.robot")


def slew(cur: float, tgt: float, up: float, down: float, dt: float) -> float:
    """Asymmetric slew rate limiter for smooth acceleration and crisp stopping."""
    # If increasing magnitude in same direction, use acceleration rate; else deceleration rate
    rate = up if abs(tgt) > abs(cur) and tgt * cur >= 0 else down
    s = rate * dt

    if tgt > cur:
        val = min(tgt, cur + s)
    else:
        val = max(tgt, cur - s)

    # Snap to exactly 0 when target is zero and close to eliminate creep
    if abs(tgt) < 1e-4 and abs(val) < max(1.0, s):
        return 0.0

    return val


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
        self.left_dir = -1 if bool(motors_cfg.get("invert_left", False)) else 1
        self.right_dir = -1 if bool(motors_cfg.get("invert_right", True)) else 1
        self.motor_type = motors_cfg.get("type", "ddsm115")

        self.driver = DDSMDriver(
            port=serial_cfg.get("port", "/dev/ttyUSB0"),
            baudrate=int(serial_cfg.get("baudrate", 115200)),
            timeout=float(serial_cfg.get("timeout", 0.02)),
            dry_run=bool(config.get("dry_run", False)),
            motor_type=self.motor_type,
        )

        # 2. Kinematics & Ramping
        robot_cfg = config.get("robot", {})
        self.turn_ratio = float(robot_cfg.get("turn_ratio", 0.65))
        self.turn_dir = int(robot_cfg.get("turn_dir", 1))
        self.accel_rate = float(robot_cfg.get("accel_rate", 1500.0))    # RPM / s
        self.decel_rate = float(robot_cfg.get("decel_rate", 1000.0))   # RPM / s on stick release
        self.brake_rate = float(robot_cfg.get("brake_rate", 3000.0))   # RPM / s on B button

        # 3. Gamepad Configuration
        gp_cfg = config.get("gamepad", {})
        self.gamepad = F710Gamepad(
            device_path=gp_cfg.get("device_path", None),
            deadzone=float(gp_cfg.get("deadzone", 0.10)),
            speed_levels=gp_cfg.get("speed_levels", [50.0, 100.0, 150.0, 200.0]),
            default_speed_level=int(gp_cfg.get("default_speed_level", 1)),
        )

        # 4. Battery / UPS HAT Configuration
        ups_cfg = config.get("ups_hat", {})
        self.ups = WaveshareUPSMonitor(
            i2c_bus=int(ups_cfg.get("i2c_bus", 1)),
            i2c_address=int(str(ups_cfg.get("i2c_address", "0x41")), 16),
            battery_type=ups_cfg.get("battery_type", "ups_2s"),
            enabled=bool(ups_cfg.get("enabled", True)),
        )

        # 5. Timing
        control_cfg = config.get("control", {})
        self.loop_hz = float(control_cfg.get("loop_hz", 50.0))
        self.telemetry_interval = float(control_cfg.get("telemetry_interval", 0.25))

        # Runtime states
        self.cur_left_rpm = 0.0
        self.cur_right_rpm = 0.0
        self.last_print_time = 0.0

    def start(self):
        """Initialize all subsystems and run control loop."""
        logger.info("Initializing MIDI Mobile Manipulator Controller...")

        # Setup OS signal handlers
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        # Connect RS485 serial bus
        if not self.driver.connect():
            logger.error("Could not connect to RS485 bus on %s. Exiting.", self.driver.port)
            sys.exit(1)

        # Set motors to Velocity Mode (0x02)
        logger.info("Setting DDSM motors (ID %d, ID %d) to Velocity Mode...", self.left_id, self.right_id)
        self.driver.set_mode(self.left_id, DDSMDriver.MODE_VELOCITY)
        time.sleep(0.05)
        self.driver.set_mode(self.right_id, DDSMDriver.MODE_VELOCITY)
        time.sleep(0.05)

        # Connect Gamepad
        logger.info("Searching for Logitech Gamepad...")
        self.gamepad.connect()

        # Initialize UPS HAT
        self.ups.init()

        logger.info("Ready! Running at %.0f Hz. (Left Stick: Fwd/Rev | Right Stick: Steer | LB: Speed+ | RB: Speed- | B: Brake)", self.loop_hz)
        self.running = True
        self._run_loop()

    def _run_loop(self):
        """Zero-latency synchronous control loop."""
        period = 1.0 / self.loop_hz
        t_prev = time.time()

        while self.running:
            now = time.time()
            dt = max(0.001, min(0.1, now - t_prev))
            t_prev = now

            # 1. Synchronous Gamepad Poll (instant event drain)
            gp = self.gamepad.poll()

            if gp.button_start_pressed:
                logger.info("START button pressed. Exiting...")
                break

            if gp.connected:
                fwd = gp.forward
                turn = gp.turn * self.turn_ratio * self.turn_dir
            else:
                fwd = 0.0
                turn = 0.0

            # 2. Twist Mixing (Forward + Turn)
            l = fwd + turn
            r = fwd - turn
            scale = max(1.0, abs(l), abs(r))
            vmax = gp.speed_rpm

            hedef_l = (l / scale) * vmax
            hedef_r = (r / scale) * vmax

            # 3. Brake handling (B button held)
            fren = gp.connected and gp.brake
            if fren:
                hedef_l = 0.0
                hedef_r = 0.0
                dec = self.brake_rate
            else:
                dec = self.decel_rate

            # 4. Asymmetric Slew Rate Ramping
            self.cur_left_rpm = slew(self.cur_left_rpm, hedef_l, self.accel_rate, dec, dt)
            self.cur_right_rpm = slew(self.cur_right_rpm, hedef_r, self.accel_rate, dec, dt)

            # 5. Send velocity commands to DDSM motors
            # Each call transmits [ID, 0x64, Speed_H, Speed_L, 0, 0, 1, Brake, 0, CRC]
            # and immediately reads the 10-byte feedback frame
            fb_l = self.driver.set_velocity(
                self.left_id,
                self.left_dir * self.cur_left_rpm,
                acceleration=1,
                brake=fren,
            )
            time.sleep(0.002)  # 2ms transceiver turnaround
            fb_r = self.driver.set_velocity(
                self.right_id,
                self.right_dir * self.cur_right_rpm,
                acceleration=1,
                brake=fren,
            )

            # 6. Periodic Status Print
            if now - self.last_print_time >= self.telemetry_interval:
                self.last_print_time = now
                self._print_status(gp, fren, fb_l, fb_r)

            # 7. Sleep remainder of loop interval
            rest = period - (time.time() - now)
            if rest > 0:
                time.sleep(rest)

    def _print_status(self, gp: GamepadState, fren: bool, fb_l: Optional[MotorFeedback], fb_r: Optional[MotorFeedback]):
        l_act = f"{fb_l.velocity_rpm:+4.0f}" if fb_l else "----"
        r_act = f"{fb_r.velocity_rpm:+4.0f}" if fb_r else "----"
        conn_str = "CONNECTED" if gp.connected else "NO GAMEPAD"
        brake_str = " | BRAKE" if fren else ""

        sys.stdout.write(
            f"\rGP: {conn_str:<10} | Fwd: {gp.forward:+4.2f} Steer: {gp.turn:+4.2f} | "
            f"Cmd: L={int(self.cur_left_rpm):+4d} R={int(self.cur_right_rpm):+4d} RPM | "
            f"Act: L={l_act} R={r_act} RPM | Gear: {gp.speed_rpm:.0f} RPM (Lv {gp.speed_level+1}){brake_str}   "
        )
        sys.stdout.flush()

    def _handle_signal(self, signum, frame):
        logger.info("\nSignal %d received. Stopping robot safely...", signum)
        self.stop()
        sys.exit(0)

    def stop(self):
        """Smoothly bring motors to halt and close bus."""
        self.running = False
        print("\nStopping robot...")

        # Smooth decelerate to 0
        t0 = time.time()
        while (abs(self.cur_left_rpm) > 1 or abs(self.cur_right_rpm) > 1) and time.time() - t0 < 1.0:
            self.cur_left_rpm = slew(self.cur_left_rpm, 0.0, 0.0, self.brake_rate, 0.02)
            self.cur_right_rpm = slew(self.cur_right_rpm, 0.0, 0.0, self.brake_rate, 0.02)
            self.driver.set_velocity(self.left_id, self.left_dir * self.cur_left_rpm, acceleration=1)
            time.sleep(0.002)
            self.driver.set_velocity(self.right_id, self.right_dir * self.cur_right_rpm, acceleration=1)
            time.sleep(0.02)

        # Final 0 RPM stop
        try:
            self.driver.emergency_brake(self.left_id)
            time.sleep(0.002)
            self.driver.emergency_brake(self.right_id)
        except Exception:
            pass

        self.gamepad.close()
        self.driver.close()
        print("Motors halted safely. RS485 bus closed.")
