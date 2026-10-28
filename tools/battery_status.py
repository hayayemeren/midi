#!/usr/bin/env python3
"""Waveshare UPS HAT Battery Telemetry Reader.

Monitors battery voltage, current draw, power consumption, and remaining percentage.
"""

import argparse
import sys
import time

sys.path.insert(0, ".")
from src.ups_battery import WaveshareUPSMonitor


def main():
    parser = argparse.ArgumentParser(description="Waveshare UPS HAT Battery Monitor")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus (default: 1)")
    parser.add_argument("--address", default="0x41", help="I2C address (default: 0x41)")
    parser.add_argument("--type", choices=["ups_2s", "makita_18v"], default="ups_2s", help="Battery type")
    parser.add_argument("-n", "--interval", type=float, default=1.0, help="Poll interval in seconds")
    args = parser.parse_args()

    addr = int(args.address, 16)
    monitor = WaveshareUPSMonitor(
        i2c_bus=args.bus,
        i2c_address=addr,
        battery_type=args.type,
        enabled=True,
    )

    if not monitor.init():
        print(f"Could not initialize UPS monitor on I2C bus {args.bus}, address 0x{addr:02X}.")
        print("Check if I2C is enabled (raspi-config -> Interface Options -> I2C).")
        sys.exit(1)

    print("=" * 60)
    print("        WAVESHARE UPS HAT / BATTERY TELEMETRY          ")
    print("=" * 60)
    print(f" I2C Bus: {args.bus} | Address: 0x{addr:02X} | Mode: {args.type}")
    print(" Press Ctrl+C to stop.")
    print("-" * 60)

    try:
        while True:
            telemetry = monitor.read()
            if telemetry.connected:
                charge_state = "CHARGING" if telemetry.is_charging else "DISCHARGING"
                print(
                    f"\rVoltage: {telemetry.bus_voltage_v:5.2f} V | "
                    f"Current: {telemetry.current_ma:7.1f} mA | "
                    f"Power: {telemetry.power_mw/1000:5.2f} W | "
                    f"SoC: {telemetry.percentage:3.0f}% | "
                    f"[{charge_state}]   ",
                    end="",
                    flush=True,
                )
            else:
                print("\r[WARNING] Unable to read from INA219. Retrying...   ", end="", flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nExiting battery monitor.")


if __name__ == "__main__":
    main()
