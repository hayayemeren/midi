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
from typing import Dict, List, Optional

logger = logging.getLogger("midi.gamepad")

# Linux joystick event constants
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


@dataclass
class GamepadState:
    """Thread-safe snapshot of gamepad control inputs."""
    # Normalized sticks (-1.0 to +1.0)
    left_stick_x: float = 0.0
    left_stick_y: float = 0.0   # Forward (+1.0) / Backward (-1.0)
    right_stick_x: float = 0.0  # Turn Left (-1.0) / Turn Right (+1.0)
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
    button_lb: bool = False  # Left Bumper (Speed UP)
    button_rb: bool = False  # Right Bumper (Speed DOWN)
    button_back: bool = False
    button_start: bool = False
    button_guide: bool = False

    # State metadata
    connected: bool = False
    emergency_stop_latched: bool = False
    speed_level: int = 3
    max_speed_levels: int = 5
    speed_multiplier: float = 0.60
    speed_mode: str = "Level 3 (60%)"
    last_event_time: float = 0.0


class F710Gamepad:
    """Logitech F710 Wireless Gamepad Controller."""

    def __init__(
        self,
        device_path: Optional[str] = None,
        deadzone: float = 0.10,
        expo: float = 1.4,
        require_deadman: bool = False,
        speed_levels: Optional[List[float]] = None,
        default_speed_level: int = 3,
    ):
        """Args:
            device_path: Explicit device path (e.g. /dev/input/js0). If None, auto-detects.
            deadzone: Analog stick deadband (0.0 to 1.0).
            expo: Exponential curve factor for smooth low-speed control.
            require_deadman: If True, Left Bumper (LB) must be held down to drive.
            speed_levels: List of speed scale multipliers (e.g. [0.2, 0.4, 0.6, 0.8, 1.0]).
            default_speed_level: Initial speed level (1-indexed).
        """
        self.device_path = device_path
        self.deadzone = deadzone
        self.expo = expo
        self.require_deadman = require_deadman

        self.speed_levels = speed_levels or [0.20, 0.40, 0.60, 0.80, 1.00]
        self.default_speed_level = max(1, min(len(self.speed_levels), default_speed_level))

        self._state = GamepadState()
        self._state.max_speed_levels = len(self.speed_levels)
        self._state.speed_level = self.default_speed_level
        self._state.speed_multiplier = self.speed_levels[self.default_speed_level - 1]
        self._state.speed_mode = f"Level {self._state.speed_level} ({self._state.speed_multiplier*100:.0f}%)"

        self._prev_lb = False
        self._prev_rb = False

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

    def speed_up(self):
        """Increase speed gear/level by 1 (triggered by Left Bumper)."""
        with self._lock:
            if self._state.speed_level < len(self.speed_levels):
                self._state.speed_level += 1
                self._state.speed_multiplier = self.speed_levels[self._state.speed_level - 1]
                self._state.speed_mode = f"Level {self._state.speed_level} ({self._state.speed_multiplier*100:.0f}%)"
                logger.info("[GAMEPAD] Speed UP -> %s", self._state.speed_mode)

    def speed_down(self):
        """Decrease speed gear/level by 1 (triggered by Right Bumper)."""
        with self._lock:
            if self._state.speed_level > 1:
                self._state.speed_level -= 1
                self._state.speed_multiplier = self.speed_levels[self._state.speed_level - 1]
                self._state.speed_mode = f"Level {self._state.speed_level} ({self._state.speed_multiplier*100:.0f}%)"
                logger.info("[GAMEPAD] Speed DOWN -> %s", self._state.speed_mode)

    def get_state(self) -> GamepadState:
        """Return a thread-safe copy of the current gamepad state."""
        with self._lock:
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
                speed_level=self._state.speed_level,
                max_speed_levels=self._state.max_speed_levels,
                speed_multiplier=self._state.speed_multiplier,
                speed_mode=self._state.speed_mode,
                last_event_time=self._state.last_event_time,
            )

    def get_drive_command(self) -> tuple[float, float, bool]:
        """Compute drive throttle and steering commands from current state.
        
        Controls:
          - Left Stick Y  : Forward / Backward (linear velocity v)
          - Right Stick X : Left / Right (angular velocity omega)
          - Speed Scaling : Controlled by LB (Speed Up) and RB (Speed Down)
        
        Returns:
            Tuple of (linear_throttle [-1..1], angular_steering [-1..1], is_active [bool])
        """
        state = self.get_state()

        # Safety checks
        if not state.connected or state.emergency_stop_latched:
            return 0.0, 0.0, False

        # Deadman check (if enabled)
        if self.require_deadman and not state.button_lb:
            return 0.0, 0.0, False

        multiplier = state.speed_multiplier

        # Left stick Y = Forward / Backward
        # Right stick X = Left / Right Steering
        linear = state.left_stick_y * multiplier
        # Invert right_stick_x: pushing LEFT (-1) produces positive angular speed (CCW turn)
        angular = -state.right_stick_x * multiplier

        # Allow D-Pad for fine inching (uses minimum crawl speed)
        if abs(linear) < 0.05 and abs(angular) < 0.05:
            min_speed = self.speed_levels[0]
            if state.dpad_y != 0.0:
                linear = state.dpad_y * min_speed
            if state.dpad_x != 0.0:
                angular = -state.dpad_x * min_speed

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
            self._state.left_stick_x = 0.0
            self._state.left_stick_y = 0.0
            self._state.right_stick_x = 0.0
            self._state.right_stick_y = 0.0

    def _worker_loop(self):
        """Background thread reading /dev/input/js* events with zero lag."""
        import select

        event_size = struct.calcsize("IhBB")

        while self._running:
            if self._fd is None:
                if not self._open_device():
                    time.sleep(1.0)
                    continue

            try:
                # Use select with 50ms timeout: zero CPU burn, instant wakeup on new events
                r, _, _ = select.select([self._fd], [], [], 0.05)
                if not r:
                    continue

                # Drain ALL queued events in the kernel buffer immediately
                while True:
                    try:
                        raw_data = os.read(self._fd, 1024)
                        if not raw_data:
                            break

                        for offset in range(0, len(raw_data) - event_size + 1, event_size):
                            chunk = raw_data[offset : offset + event_size]
                            event_time, value, event_type, number = struct.unpack("IhBB", chunk)
                            pure_type = event_type & ~JS_EVENT_INIT

                            if pure_type == JS_EVENT_BUTTON:
                                self._handle_button(number, bool(value))
                            elif pure_type == JS_EVENT_AXIS:
                                self._handle_axis(number, value)

                    except BlockingIOError:
                        break  # Kernel buffer completely drained to current real-time state!

            except (OSError, Exception) as e:
                logger.warning("Gamepad disconnected or read error: %s", e)
                self._close_device()
                time.sleep(1.0)

    def _handle_button(self, number: int, pressed: bool):
        """Process button press/release events."""
        with self._lock:
            self._state.last_event_time = time.time()

            # Map standard XInput / DirectInput buttons
            if number == 0:  # A button -> Reset E-Stop
                self._state.button_a = pressed
                if pressed and self._state.emergency_stop_latched:
                    self._state.emergency_stop_latched = False
                    logger.info("[GAMEPAD] E-Stop reset by button A.")
            elif number == 1:  # B button -> Emergency Stop
                self._state.button_b = pressed
                if pressed:
                    self._state.emergency_stop_latched = True
                    logger.warning("[GAMEPAD] EMERGENCY STOP triggered by button B!")
            elif number == 2:  # X button
                self._state.button_x = pressed
            elif number == 3:  # Y button
                self._state.button_y = pressed
            elif number == 4:  # LB (Left Bumper) -> SPEED UP
                self._state.button_lb = pressed
                if pressed and not self._prev_lb:
                    self.speed_up()
                self._prev_lb = pressed
            elif number == 5:  # RB (Right Bumper) -> SPEED DOWN
                self._state.button_rb = pressed
                if pressed and not self._prev_rb:
                    self.speed_down()
                self._prev_rb = pressed
            elif number == 6:  # Back button -> E-Stop
                self._state.button_back = pressed
                if pressed:
                    self._state.emergency_stop_latched = True
                    logger.warning("[GAMEPAD] EMERGENCY STOP triggered by BACK button!")
            elif number == 7:  # Start button -> Reset E-Stop
                self._state.button_start = pressed
                if pressed and self._state.emergency_stop_latched:
                    self._state.emergency_stop_latched = False
                    logger.info("[GAMEPAD] E-Stop reset by START button.")
            elif number == 8:  # Guide / Logitech button
                self._state.button_guide = pressed

    def _handle_axis(self, number: int, raw_value: int):
        """Process analog stick and trigger movements."""
        norm_val = raw_value / 32767.0
        norm_val = max(-1.0, min(1.0, norm_val))

        with self._lock:
            self._state.last_event_time = time.time()

            if number == 0:  # Left stick X (Horizontal)
                self._state.left_stick_x = self._apply_deadzone_and_expo(norm_val)
            elif number == 1:  # Left stick Y (Vertical, invert so Push Forward is +1.0)
                self._state.left_stick_y = self._apply_deadzone_and_expo(-norm_val)
            elif number == 2:  # LT (Left Trigger)
                self._state.left_trigger = (norm_val + 1.0) / 2.0
            elif number == 3:  # Right stick X (Horizontal, Steering)
                self._state.right_stick_x = self._apply_deadzone_and_expo(norm_val)
            elif number == 4:  # Right stick Y (Vertical)
                self._state.right_stick_y = self._apply_deadzone_and_expo(-norm_val)
            elif number == 5:  # RT (Right Trigger)
                self._state.right_trigger = (norm_val + 1.0) / 2.0
            elif number == 6:  # D-pad X
                self._state.dpad_x = 1.0 if norm_val > 0.5 else (-1.0 if norm_val < -0.5 else 0.0)
            elif number == 7:  # D-pad Y (invert so Up is +1.0)
                self._state.dpad_y = -1.0 if norm_val > 0.5 else (1.0 if norm_val < -0.5 else 0.0)
