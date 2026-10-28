#!/usr/bin/env python3
"""Logitech F710 Gamepad Interactive Diagnostic Visualizer.

Displays live stick positions, button states, and computed drive commands
in the terminal. Run this on Raspberry Pi to test controller pairing.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, ".")
from src.gamepad import F710Gamepad


def format_bar(val: float, width: int = 20) -> str:
    """Format -1.0..1.0 value into an ASCII center-bar."""
    center = width // 2
    pos = int(center + (val * center))
    pos = max(0, min(width - 1, pos))
    bar = ["-"] * width
    bar[center] = "|"
    bar[pos] = "#"
    return "".join(bar)


def main():
    parser = argparse.ArgumentParser(description="Logitech F710 Gamepad Test Visualizer")
    parser.add_argument("-d", "--device", default=None, help="Device path (default: auto-detect)")
    parser.add_argument("--deadman", action="store_true", help="Enable Left Bumper (LB) deadman requirement")
    args = parser.parse_args()

    gp = F710Gamepad(device_path=args.device, require_deadman=args.deadman)
    gp.start()

    print("Starting Gamepad Diagnostic Visualizer... Press Ctrl+C to exit.")
    time.sleep(0.5)

    try:
        while True:
            state = gp.get_state()
            throttle, steering, is_active = gp.get_drive_command()

            # Clear screen (ANSI escape code)
            sys.stdout.write("\033[H\033[J")
            sys.stdout.flush()

            status_str = "CONNECTED" if state.connected else "DISCONNECTED / WAITING"
            active_str = "DRIVE ACTIVE" if is_active else "DRIVE INHIBITED (E-STOP or DEADMAN)"

            print("=" * 65)
            print("       LOGITECH F710 WIRELESS GAMEPAD DIAGNOSTICS       ")
            print("=" * 65)
            print(f" Status     : {status_str} | Mode: [{state.speed_mode.upper()}]")
            print(f" Drive State: {active_str}")
            print(f" E-Stop Latched: {'YES (Press A to reset)' if state.emergency_stop_latched else 'NO'}")
            print("-" * 65)
            print(f" Left Stick X (Steer)  : [{format_bar(state.left_stick_x)}] {state.left_stick_x:+.2f}")
            print(f" Left Stick Y (Drive)  : [{format_bar(state.left_stick_y)}] {state.left_stick_y:+.2f}")
            print(f" Right Stick X         : [{format_bar(state.right_stick_x)}] {state.right_stick_x:+.2f}")
            print(f" Right Stick Y         : [{format_bar(state.right_stick_y)}] {state.right_stick_y:+.2f}")
            print("-" * 65)
            print(f" Triggers  : LT={state.left_trigger:.2f} | RT={state.right_trigger:.2f}")
            print(f" D-Pad     : X={state.dpad_x:+.0f} | Y={state.dpad_y:+.0f}")
            print("-" * 65)
            print(" Buttons:")
            buttons = [
                ("A (Reset)", state.button_a),
                ("B (E-Stop)", state.button_b),
                ("X (Norm)", state.button_x),
                ("Y (Turbo)", state.button_y),
                ("LB (Deadman)", state.button_lb),
                ("RB (Crawl)", state.button_rb),
                ("BACK", state.button_back),
                ("START", state.button_start),
            ]
            btn_str = " | ".join(f"{name}: {'[ON]' if active else 'off'}" for name, active in buttons)
            print(f" {btn_str}")
            print("-" * 65)
            print(f" OUTPUT COMMANDS -> Linear: {throttle:+.2f} | Angular: {steering:+.2f}")
            print("=" * 65)
            print("Controls Guide:")
            print(" - Left Stick: Push UP/DOWN to drive, LEFT/RIGHT to steer")
            print(" - Button B or BACK: Instant Emergency Stop (cuts motors)")
            print(" - Button A or START: Clear Emergency Stop")
            print(" - Button X / Y / RB: Switch speed (Normal / Turbo / Crawl)")

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting visualizer.")
    finally:
        gp.stop()


if __name__ == "__main__":
    main()
