"""Logitech Wireless Gamepad F710 Driver using Python evdev.

Provides synchronous, zero-latency polling matching hardware evdev capabilities.
Supports both XInput (X mode) and DirectInput (D mode).
"""

from __future__ import annotations

import logging
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

logger = logging.getLogger("midi.gamepad")

try:
    import evdev
    from evdev import ecodes as E
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False
    E = None

# Gamepad Button Codes for XInput and DirectInput modes
TUS_FREN  = {E.BTN_EAST, E.BTN_THUMB2} if E else {0x131, 0x121}  # B button
TUS_LB    = {E.BTN_TL, E.BTN_TOP2} if E else {0x136, 0x124}      # Left Bumper (Speed UP)
TUS_RB    = {E.BTN_TR, E.BTN_PINKIE} if E else {0x137, 0x125}    # Right Bumper (Speed DOWN)
TUS_START = {E.BTN_START, E.BTN_BASE4} if E else {0x13b, 0x129}  # START button


@dataclass
class GamepadState:
    """Snapshot of gamepad inputs."""
    forward: float = 0.0          # -1.0 (Reverse) to +1.0 (Forward)
    turn: float = 0.0             # -1.0 (Right/CW) to +1.0 (Left/CCW)
    brake: bool = False           # True if B is pressed
    button_lb_pressed: bool = False
    button_rb_pressed: bool = False
    button_start_pressed: bool = False
    connected: bool = False
    device_name: str = "None"
    speed_level: int = 2          # Current speed level index
    speed_rpm: float = 100.0


class F710Gamepad:
    """Synchronous Gamepad reader using evdev (Zero-lag)."""

    def __init__(
        self,
        device_path: Optional[str] = None,
        deadzone: float = 0.10,
        speed_levels: Optional[List[float]] = None,
        default_speed_level: int = 2,
    ):
        self.device_path = device_path
        self.deadzone = deadzone
        self.speed_levels = speed_levels or [50.0, 100.0, 150.0, 200.0]
        self.speed_level = max(0, min(len(self.speed_levels) - 1, default_speed_level))

        self.dev = None
        self.ax: Dict[int, Tuple[int, int]] = {}     # code -> (min, max)
        self.val: Dict[int, int] = {}                # code -> current raw value
        self.btn: Set[int] = set()                   # currently held buttons
        self.pressed: List[int] = []                 # buttons pressed this cycle
        self.turn_code: Optional[int] = None
        self.connected = False
        self.device_name = "Not Connected"
        self._last_reconnect_attempt = 0.0

    @staticmethod
    def inspect(path: str):
        """Check if device has necessary sticks and keys for Logitech gamepad."""
        if not EVDEV_AVAILABLE:
            return None, {}, set(), False, "evdev library not installed"
        try:
            d = evdev.InputDevice(path)
            caps = dict(d.capabilities(absinfo=True))
            axes = {c: info for c, info in caps.get(E.EV_ABS, [])}
            keys = set(caps.get(E.EV_KEY, []))
            pad_keys = any(0x120 <= k <= 0x13F for k in keys)

            if not (E.ABS_X in axes and E.ABS_Y in axes):
                return d, axes, keys, False, "Left stick (ABS_X/ABS_Y) missing"
            if not pad_keys:
                return d, axes, keys, False, "Gamepad button keys missing"
            if not (E.ABS_RX in axes or E.ABS_Z in axes):
                return d, axes, keys, False, "Right stick (ABS_RX/ABS_Z) missing"

            return d, axes, keys, True, "OK"
        except Exception as e:
            return None, {}, set(), False, str(e)

    def connect(self, force_path: Optional[str] = None, verbose: bool = False) -> bool:
        """Find and open the Logitech F710 device."""
        if not EVDEV_AVAILABLE:
            logger.warning("evdev library is not available.")
            return False

        paths = [force_path] if force_path else evdev.list_devices()
        for path in paths:
            try:
                d, axes, keys, ok, why = self.inspect(path)
            except Exception as e:
                if verbose:
                    logger.debug("Device %s check failed: %s", path, e)
                continue

            if not ok:
                if verbose:
                    logger.debug("Skipping %s: %s", path, why)
                continue

            # Right stick X: XInput is ABS_RX, DirectInput is ABS_Z
            turn = E.ABS_RX if E.ABS_RX in axes else E.ABS_Z
            self.dev = d
            self.turn_code = turn
            self.ax = {c: (i.min, i.max) for c, i in axes.items()}
            self.val = {c: i.value for c, i in axes.items()}
            self.btn.clear()
            self.connected = True
            self.device_name = d.name

            mode_str = "XInput (X)" if turn == E.ABS_RX else "DirectInput (D)"
            logger.info("Gamepad connected: %s on %s (%s)", d.name, path, mode_str)
            return True

        self.connected = False
        return False

    def norm(self, code: int) -> float:
        """Normalize analog axis value using hardware calibrated range."""
        lo, hi = self.ax.get(code, (-32767, 32767))
        center = (lo + hi) / 2.0
        half_span = (hi - lo) / 2.0 or 1.0

        v = self.val.get(code, int(center))
        x = (v - center) / half_span
        x = max(-1.0, min(1.0, x))

        if abs(x) < self.deadzone:
            return 0.0

        # Continuous scale starting immediately from 0 beyond deadzone
        return math.copysign((abs(x) - self.deadzone) / (1.0 - self.deadzone), x)

    def poll(self) -> GamepadState:
        """Synchronously drain all pending kernel events and return real-time state.
        
        Zero queue lag: reads all events with read_one() until None.
        """
        self.pressed = []
        now = time.time()

        if self.dev is None:
            if now - self._last_reconnect_attempt > 1.0:
                self._last_reconnect_attempt = now
                self.connect(self.device_path)
            if self.dev is None:
                return GamepadState(
                    connected=False,
                    speed_level=self.speed_level,
                    speed_rpm=self.speed_levels[self.speed_level],
                )

        try:
            # Drain all pending events in kernel buffer
            while True:
                ev = self.dev.read_one()
                if ev is None:
                    break
                if ev.type == E.EV_ABS:
                    self.val[ev.code] = ev.value
                elif ev.type == E.EV_KEY:
                    if ev.value == 1:
                        self.btn.add(ev.code)
                        self.pressed.append(ev.code)
                    elif ev.value == 0:
                        self.btn.discard(ev.code)

        except OSError:
            logger.warning("Gamepad disconnected!")
            self.dev = None
            self.connected = False
            return GamepadState(
                connected=False,
                speed_level=self.speed_level,
                speed_rpm=self.speed_levels[self.speed_level],
            )

        # Handle speed level adjustments: LB = speed up, RB = speed down
        for btn in self.pressed:
            if btn in TUS_LB:
                self.speed_level = min(len(self.speed_levels) - 1, self.speed_level + 1)
                logger.info("[GAMEPAD] Speed UP (LB): Level %d (%.0f RPM)", self.speed_level + 1, self.speed_levels[self.speed_level])
            elif btn in TUS_RB:
                self.speed_level = max(0, self.speed_level - 1)
                logger.info("[GAMEPAD] Speed DOWN (RB): Level %d (%.0f RPM)", self.speed_level + 1, self.speed_levels[self.speed_level])

        # Compute analog stick values:
        # Left Stick Y: Push Forward = positive (+1), Pull Back = negative (-1)
        raw_fwd = -self.norm(E.ABS_Y)
        # Right Stick X: Push Left = positive (+1 CCW turn), Push Right = negative (-1 CW turn)
        raw_turn = -self.norm(self.turn_code) if self.turn_code else 0.0

        brake_active = bool(TUS_FREN & self.btn)
        start_pressed = bool(TUS_START & set(self.pressed))

        return GamepadState(
            forward=raw_fwd,
            turn=raw_turn,
            brake=brake_active,
            button_lb_pressed=bool(TUS_LB & set(self.pressed)),
            button_rb_pressed=bool(TUS_RB & set(self.pressed)),
            button_start_pressed=start_pressed,
            connected=True,
            device_name=self.device_name,
            speed_level=self.speed_level,
            speed_rpm=self.speed_levels[self.speed_level],
        )

    def close(self):
        if self.dev:
            try:
                self.dev.close()
            except Exception:
                pass
            self.dev = None
            self.connected = False
