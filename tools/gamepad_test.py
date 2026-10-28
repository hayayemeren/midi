#!/usr/bin/env python3
"""Logitech F710 Gamepad Interactive Diagnostic Visualizer (evdev based).

Displays live stick positions, button states, and speed gears in the terminal.
Includes --list option to inspect all connected input devices.
"""

import argparse
import sys
import time

sys.path.insert(0, ".")
from src.gamepad import F710Gamepad, EVDEV_AVAILABLE


def format_bar(val: float, width: int = 24) -> str:
    """Format -1.0..1.0 value into an ASCII center-bar."""
    center = width // 2
    pos = int(center + (val * center))
    pos = max(0, min(width - 1, pos))
    bar = ["-"] * width
    bar[center] = "|"
    bar[pos] = "#"
    return "".join(bar)


def list_devices():
    """List all connected evdev input devices for troubleshooting."""
    if not EVDEV_AVAILABLE:
        print("evdev library is not installed! Run: sudo apt install python3-evdev")
        return
    import evdev
    from evdev import ecodes as E
    devices = evdev.list_devices()
    if not devices:
        print("No input devices found. Check if your user is in the 'input' group:")
        print("  sudo usermod -a -G input $USER")
        return

    print("Found input devices:")
    for path in devices:
        try:
            d, axes, keys, ok, why = F710Gamepad.inspect(path)
            ax_names = ", ".join(str(E.ABS.get(c, c)) for c in sorted(axes.keys()))
            status = " [READY FOR F710]" if ok else f" (Not suitable: {why})"
            print(f"  {path}: {d.name}{status}")
            if axes:
                print(f"      Axes: {ax_names}")
        except Exception as e:
            print(f"  {path}: Error inspecting ({e})")


def main():
    parser = argparse.ArgumentParser(description="Logitech F710 Gamepad Diagnostic Visualizer")
    parser.add_argument("-d", "--device", default=None, help="Force specific device path (e.g. /dev/input/event3)")
    parser.add_argument("--list", action="store_true", help="List all available input devices and exit")
    args = parser.parse_args()

    if args.list:
        list_devices()
        return

    gp = F710Gamepad(device_path=args.device)
    gp.connect(verbose=True)

    print("Starting Gamepad Diagnostic Visualizer... Press Ctrl+C or START to exit.")
    time.sleep(0.5)

    try:
        while True:
            state = gp.poll()

            # Clear screen (ANSI escape code)
            sys.stdout.write("\033[H\033[J")
            sys.stdout.flush()

            status_str = f"CONNECTED ({state.device_name})" if state.connected else "SEARCHING FOR GAMEPAD..."

            print("=" * 65)
            print("       LOGITECH F710 WIRELESS GAMEPAD DIAGNOSTICS       ")
            print("=" * 65)
            print(f" Status     : {status_str}")
            print(f" Speed Gear : Level {state.speed_level + 1} ({state.speed_rpm:.0f} RPM)")
            print(f" Brake (B)  : {'[PRESSED]' if state.brake else 'off'}")
            print("-" * 65)
            print(f" Left Stick Y (Drive Fwd/Rev) : [{format_bar(state.forward)}] {state.forward:+.2f}")
            print(f" Right Stick X (Steer L/R)    : [{format_bar(state.turn)}] {state.turn:+.2f}")
            print("-" * 65)
            print(" Controls Guide:")
            print("  - Left Stick Y: Push Forward to drive forward, Pull Back to reverse")
            print("  - Right Stick X: Push Left to steer CCW, Push Right to steer CW")
            print("  - Left Bumper (LB): Speed UP (+ gear / RPM)")
            print("  - Right Bumper (RB): Speed DOWN (- gear / RPM)")
            print("  - Button B: Hold for Brake")
            print("  - START: Exit program")
            print("=" * 65)

            if state.button_start_pressed:
                print("\nSTART button pressed. Exiting visualizer.")
                break

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting visualizer.")
    finally:
        gp.close()


if __name__ == "__main__":
    main()
