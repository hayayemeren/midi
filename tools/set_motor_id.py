#!/usr/bin/env python3
"""DDSM Motor RS485 ID Programming Utility.

Use this tool to assign unique IDs (e.g. ID 1 for Left motor, ID 2 for Right motor).

CRITICAL WARNING:
Connect ONLY ONE motor to the RS485 adapter when running this tool!
If both motors are connected simultaneously, both will be programmed to the same ID!
"""

import argparse
import sys
import time

sys.path.insert(0, ".")
from src.ddsm import DDSMDriver


def main():
    parser = argparse.ArgumentParser(description="Program DDSM Motor RS485 ID")
    parser.add_argument(
        "-p", "--port",
        default="/dev/ttyUSB0",
        help="RS485 Serial port path (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "-b", "--baudrate",
        type=int,
        default=115200,
        help="Serial baudrate (default: 115200)"
    )
    parser.add_argument(
        "-id", "--new-id",
        type=int,
        required=True,
        help="New ID to assign to the connected motor (1 to 254)"
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip safety confirmation prompt"
    )

    args = parser.parse_args()

    if not (1 <= args.new_id <= 254):
        print(f"Error: Motor ID must be between 1 and 254. Got {args.new_id}")
        sys.exit(1)

    print("=" * 60)
    print("           DDSM MOTOR ID PROGRAMMING UTILITY           ")
    print("=" * 60)
    print("IMPORTANT SAFETY NOTICE:")
    print(" Ensure ONLY ONE motor is wired to the RS485 bus right now.")
    print(" If multiple motors are connected, ALL of them will adopt this ID!")
    print(f" Target Serial Port : {args.port}")
    print(f" Target Motor ID    : {args.new_id}")
    print("=" * 60)

    if not args.yes:
        confirm = input(f"Proceed with programming motor to ID {args.new_id}? (yes/no): ").strip().lower()
        if confirm != "yes":
            print("Operation aborted by user.")
            sys.exit(0)

    driver = DDSMDriver(port=args.port, baudrate=args.baudrate)
    if not driver.connect():
        print(f"Failed to connect to serial port {args.port}. Check permissions or cable.")
        sys.exit(1)

    try:
        print(f"\nSending ID assignment command (AA 55 53 {args.new_id:02X} ...) 5 times...")
        success = driver.set_motor_id(args.new_id)
        if success:
            print("Commands transmitted successfully.")
            print("Waiting 1 second for motor non-volatile memory write...")
            time.sleep(1.0)

            print(f"Testing feedback query with new ID {args.new_id}...")
            fb = driver.query_feedback(args.new_id)
            if fb:
                print(f"[SUCCESS] Motor responded to ID {fb.motor_id}!")
                print(f"  Mode: {fb.mode}, Speed: {fb.velocity_rpm} RPM, Current: {fb.current_amps} A")
            else:
                print("[NOTE] Motor commands sent. If the motor does not respond immediately,")
                print("       please POWER CYCLE the motor (disconnect & reconnect Makita power).")
                print("       DDSM motors persist their new ID across power cycles.")
        else:
            print("[ERROR] Failed to transmit ID setting commands.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
