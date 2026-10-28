"""Logitech Wireless Gamepad F710 / Generic Joystick Interface.

Reads joystick and button events from Linux /dev/input/js* interface without
requiring X11, SDL, or external heavy GUI frameworks.
Supports XInput and DirectInput modes, auto-reconnect, deadband filtering,
and safety deadman/emergency stop triggers.
"""

from __future__ import annotations

import glob
import logging
import math
import os
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

logger = logging.getLogger("midi.gamepad")

# Linux joystick event constants
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


@dataclass
class GamepadState:
    """Thread-safe snapshot of gamepad control inputs."""
    # Normalized sticks (-1.0 to +1.0)
    left_stick_x: float = 0.0   # Left (-1.0) to Right (+1.0)
    left_stick_y: float = 0.0   # Forward (+1.0) to Backward (-1.0)
    right_stick_x: float = 0.0
    right_stick_y: float = 0.0

    # Triggers (-1.0 unpressed to +1.0 fully pressed, or 0.0 to 1.0)
    left_trigger: float = 0.0
    right_trigger: float = 0.0

    # D-pad (-1.0, 0.0, +1.0)
    dpad_x: float = 0.0
    dpad_y: float = 0.0

    # Buttons (True when pressed)
    button_a: bool = False
    button_b: bool = False
    button_x: bool = False
    button_y: bool = False
    button_lb: bool = False  # Left Bumper
    button_rb: bool = False  # Right Bumper
    button_back: bool = False
    button_start: bool = False
    button_guide: bool = False

    # State metadata
    connected: bool = False
    emergency_stop_latched: bool = False
    speed_mode: str = "normal"  # "crawl", "normal", "turbo"
    last_event_time: float = 0.0


class F710Gamepad:
    """Logitech F710 Wireless Gamepad Controller."""

    def __init__(
        self,
        device_path: Optional[str] = None,
        deadzone: float = 0.10,
        expo: float = 1.4,
        require_deadman: bool = False,
        speed_scale_crawl: float = 0.25,
        speed_scale_normal: float = 0.65,
        speed_scale_turbo: float = 1.0,
    ):
        """Args:
            device_path: Explicit device path (e.g. /dev/input/js0). If None, auto-detects.
            deadzone: Analog stick deadband (0.0 to 1.0).
            expo: Exponential curve factor for smooth low-speed control.
            require_deadman: If True, Left Bumper (LB) must be held down to drive.
            speed_scale_crawl: Fraction of max speed in crawl mode (e.g. 0.25).
            speed_scale_normal: Fraction of max speed in normal mode (e.g. 0.65).
            speed_scale_turbo: Fraction of max speed in turbo mode (e.g. 1.0).
        """
        self.device_path = device_path
        self.deadzone = deadzone
        self.expo = expo
        self.require_deadman = require_deadman
        self.speed_scale_crawl = speed_scale_crawl
        self.speed_scale_normal = speed_scale_normal
        self.speed_scale_turbo = speed_scale_turbo

        self._state = GamepadState()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._fd: Optional[int] = None
        self._device_name: str = "Unknown"
        self._is_xinput: bool = True

    def start(self):
        """Start background reader thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, name="GamepadWorker", daemon=True)
        self._thread.start()
        logger.info("Gamepad worker thread started.")

    def stop(self):
        """Stop background reader thread."""
        self._running = False
        self._close_device()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logger.info("Gamepad worker thread stopped.")

    def get_state(self) -> GamepadState:
        """Return a thread-safe copy of the current gamepad state."""
        with self._lock:
            # Create a shallow copy
            return GamepadState(
                left_stick_x=self._state.left_stick_x,
                left_stick_y=self._state.left_stick_y,
                right_stick_x=self._state.right_stick_x,
                right_stick_y=self._state.right_stick_y,
                left_trigger=self._state.left_trigger,
                right_trigger=self._state.right_trigger,
                dpad_x=self._state.dpad_x,
                dpad_y=self._state.dpad_y,
                button_a=self._state.button_a,
                button_b=self._state.button_b,
                button_x=self._state.button_x,
                button_y=self._state.button_y,
                button_lb=self._state.button_lb,
                button_rb=self._state.button_rb,
                button_back=self._state.button_back,
                button_start=self._state.button_start,
                button_guide=self._state.button_guide,
                connected=self._state.connected,
                emergency_stop_latched=self._state.emergency_stop_latched,
                speed_mode=self._state.speed_mode,
                last_event_time=self._state.last_event_time,
            )

    def get_drive_command(self) -> tuple[float, float, bool]:
        """Compute drive throttle and steering commands from current state.
        
        Returns:
            Tuple of (linear_throttle [-1..1], angular_steering [-1..1], is_active [bool])
            Where is_active is True only if controller is connected, safety checks pass,
            and E-Stop is not latched.
        """
        state = self.get_state()

        # Safety checks
        if not state.connected or state.emergency_stop_latched:
            return 0.0, 0.0, False

        # Deadman check (if enabled)
        if self.require_deadman and not state.button_lb:
            return 0.0, 0.0, False

        # Determine speed multiplier
        if state.speed_mode == "crawl" or state.button_rb:
            multiplier = self.speed_scale_crawl
        elif state.speed_mode == "turbo" or state.right_trigger > 0.5:
            multiplier = self.speed_scale_turbo
        else:
            multiplier = self.speed_scale_normal

        # Left stick Y = Linear throttle (forward is positive)
        # Left stick X = Steering (turn left is positive)
        linear = state.left_stick_y * multiplier
        angular = -state.left_stick_x * multiplier  # Invert so push-left produces positive turn (CCW)

        # Allow D-Pad for precision inching
        if abs(linear) < 0.05 and abs(angular) < 0.05:
            if state.dpad_y != 0.0:
                linear = state.dpad_y * self.speed_scale_crawl
            if state.dpad_x != 0.0:
                angular = -state.dpad_x * self.speed_scale_crawl

        return linear, angular, True

    def reset_estop(self):
        """Clear latched emergency stop state."""
        with self._lock:
            self._state.emergency_stop_latched = False
            logger.info("Emergency Stop latch cleared.")

    def _apply_deadzone_and_expo(self, raw_val: float) -> float:
        """Apply deadband filtering and exponential response curve."""
        if abs(raw_val) < self.deadzone:
            return 0.0
        # Normalize range outside deadzone to 0.0 - 1.0
        normalized = (abs(raw_val) - self.deadzone) / (1.0 - self.deadzone)
        # Apply exponential curve
        curved = math.pow(normalized, self.expo)
        return math.copysign(curved, raw_val)

    def _find_device(self) -> Optional[str]:
        """Locate joystick device path."""
        if self.device_path and os.path.exists(self.device_path):
            return self.device_path

        # Check by-id symlinks first for Logitech F710
        f710_candidates = glob.glob("/dev/input/by-id/*F710*") + glob.glob("/dev/input/by-id/*Logitech*")
        for cand in f710_candidates:
            if "js" in os.path.basename(cand) or "event" not in cand:
                return cand

        # Fall back to standard js devices
        js_devices = sorted(glob.glob("/dev/input/js*"))
        if js_devices:
            return js_devices[0]

        return None

    def _open_device(self) -> bool:
        """Attempt to open joystick device."""
        path = self._find_device()
        if not path:
            return False

        try:
            self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            logger.info("Opened gamepad device: %s", path)
            with self._lock:
                self._state.connected = True
                self._state.last_event_time = time.time()
            return True
        except Exception as e:
            logger.debug("Failed to open %s: %s", path, e)
            self._fd = None
            return False

    def _close_device(self):
        """Close opened file descriptor."""
        if self._fd is not None:
            try:
                os.close(self._fd)
            except Exception:
                pass
            self._fd = None
        with self._lock:
            self._state.connected = False
            # Zero stick inputs on disconnect for safety
            self._state.left_stick_x = 0.0
            self._state.left_stick_y = 0.0
            self._state.right_stick_x = 0.0
            self._state.right_stick_y = 0.0

    def _worker_loop(self):
        """Background thread reading /dev/input/js* events."""
        event_size = struct.calcsize("IhBB")

        while self._running:
            if self._fd is None:
                if not self._open_device():
                    time.sleep(1.0)
                    continue

            try:
                # Read 8-byte event
                event_data = os.read(self._fd, event_size)
                if not event_data or len(event_data) < event_size:
                    time.sleep(0.01)
                    continue

                event_time, value, event_type, number = struct.unpack("IhBB", event_data)
                # Ignore initial configuration events
                is_init = bool(event_type & JS_EVENT_INIT)
                pure_type = event_type & ~JS_EVENT_INIT

                if pure_type == JS_EVENT_BUTTON:
                    self._handle_button(number, bool(value))
                elif pure_type == JS_EVENT_AXIS:
                    self._handle_axis(number, value)

            except BlockingIOError:
                time.sleep(0.01)
            except (OSError, Exception) as e:
                logger.warning("Gamepad disconnected or read error: %s", e)
                self._close_device()
                time.sleep(1.0)

    def _handle_button(self, number: int, pressed: bool):
        """Process button press/release events."""
        with self._lock:
            self._state.last_event_time = time.time()

            # Map standard XInput / DirectInput buttons
            if number == 0:  # A button
                self._state.button_a = pressed
                if pressed and self._state.emergency_stop_latched:
                    self._state.emergency_stop_latched = False
                    logger.info("[GAMEPAD] E-Stop reset by button A.")
            elif number == 1:  # B button -> Emergency Stop
                self._state.button_b = pressed
                if pressed:
                    self._state.emergency_stop_latched = True
                    logger.warning("[GAMEPAD] EMERGENCY STOP triggered by button B!")
            elif number == 2:  # X button -> Normal speed
                self._state.button_x = pressed
                if pressed:
                    self._state.speed_mode = "normal"
                    logger.info("[GAMEPAD] Speed mode: NORMAL")
            elif number == 3:  # Y button -> Turbo speed
                self._state.button_y = pressed
                if pressed:
                    self._state.speed_mode = "turbo"
                    logger.info("[GAMEPAD] Speed mode: TURBO")
            elif number == 4:  # LB (Left Bumper) -> Deadman switch
                self._state.button_lb = pressed
            elif number == 5:  # RB (Right Bumper) -> Crawl speed
                self._state.button_rb = pressed
                if pressed:
                    self._state.speed_mode = "crawl"
                    logger.info("[GAMEPAD] Speed mode: CRAWL")
            elif number == 6:  # Back button -> E-Stop
                self._state.button_back = pressed
                if pressed:
                    self._state.emergency_stop_latched = True
                    logger.warning("[GAMEPAD] EMERGENCY STOP triggered by BACK button!")
            elif number == 7:  # Start button -> Reset E-Stop / Arm
                self._state.button_start = pressed
                if pressed and self._state.emergency_stop_latched:
                    self._state.emergency_stop_latched = False
                    logger.info("[GAMEPAD] E-Stop reset by START button.")
            elif number == 8:  # Guide / Logitech button
                self._state.button_guide = pressed

    def _handle_axis(self, number: int, raw_value: int):
        """Process analog stick and trigger movements."""
        # Convert -32767..32767 to -1.0..1.0
        norm_val = raw_value / 32767.0
        norm_val = max(-1.0, min(1.0, norm_val))

        with self._lock:
            self._state.last_event_time = time.time()

            if number == 0:  # Left stick X (Horizontal)
                self._state.left_stick_x = self._apply_deadzone_and_expo(norm_val)
            elif number == 1:  # Left stick Y (Vertical, invert so Forward is +1.0)
                self._state.left_stick_y = self._apply_deadzone_and_expo(-norm_val)
            elif number == 2:  # LT (Left Trigger) or Right Stick X (in DirectInput)
                self._state.left_trigger = (norm_val + 1.0) / 2.0
            elif number == 3:  # Right stick X
                self._state.right_stick_x = self._apply_deadzone_and_expo(norm_val)
            elif number == 4:  # Right stick Y (invert so Forward is +1.0)
                self._state.right_stick_y = self._apply_deadzone_and_expo(-norm_val)
            elif number == 5:  # RT (Right Trigger)
                self._state.right_trigger = (norm_val + 1.0) / 2.0
            elif number == 6:  # D-pad X
                self._state.dpad_x = 1.0 if norm_val > 0.5 else (-1.0 if norm_val < -0.5 else 0.0)
            elif number == 7:  # D-pad Y (invert so Up is +1.0)
                self._state.dpad_y = -1.0 if norm_val > 0.5 else (1.0 if norm_val < -0.5 else 0.0)
