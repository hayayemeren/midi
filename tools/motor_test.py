#!/usr/bin/env python3
"""DDSM Motor Diagnostic and Spin Test Tool.

Spins a motor at a safe target RPM, queries real-time feedback, and safely stops.
"""

import argparse
import sys
import time

sys.path.insert(0, ".")
from src.ddsm import DDSMDriver


def test_motor(driver: DDSMDriver, motor_id: int, target_rpm: float, duration: float):
    print(f"\n--- Testing Motor ID {motor_id} at {target_rpm} RPM for {duration}s ---")
    
    # 1. Switch to Velocity Mode
    print(f"Setting Motor {motor_id} to Velocity Mode...")
    driver.set_mode(motor_id, DDSMDriver.MODE_VELOCITY)
    time.sleep(0.05)

    # 2. Query initial status
    fb_start = driver.query_feedback(motor_id)
    if fb_start:
        print(f"Initial State: Mode={fb_start.mode}, RPM={fb_start.velocity_rpm}, Current={fb_start.current_amps}A")
    else:
        print("Warning: No response to query. Sending drive command anyway...")

    # 3. Spin loop
    start_time = time.time()
    try:
        while time.time() - start_time < duration:
            driver.set_velocity(motor_id, target_rpm, acceleration=50)
            time.sleep(0.05)
            fb = driver.query_feedback(motor_id)
            if fb:
                print(
                    f"\rRunning Motor {motor_id} | Target: {target_rpm:+.1f} RPM | "
                    f"Actual: {fb.velocity_rpm:+.1f} RPM | Current: {fb.current_amps:.2f} A | "
                    f"Err: 0x{fb.error_code:02X}   ",
                    end="",
                    flush=True,
                )
    finally:
        print("\nStopping motor and applying electric brake...")
        driver.emergency_brake(motor_id)
        time.sleep(0.1)
        driver.set_velocity(motor_id, 0.0, brake=True)
        print("Motor halted safely.")


def main():
    parser = argparse.ArgumentParser(description="DDSM Motor Diagnostic Tool")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0", help="RS485 port")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="Baudrate")
    parser.add_argument("-id", "--motor-id", type=int, default=1, help="Motor ID to test (1, 2, etc.)")
    parser.add_argument("-r", "--rpm", type=float, default=30.0, help="Target RPM (positive or negative)")
    parser.add_argument("-t", "--time", type=float, default=3.0, help="Test duration in seconds")
    parser.add_argument("--both", action="store_true", help="Test both ID 1 and ID 2 consecutively")
    parser.add_argument("--motor-type", default="ddsm115", choices=["ddsm115", "ddsm210"], help="Motor model")

    args = parser.parse_args()

    driver = DDSMDriver(
        port=args.port,
        baudrate=args.baudrate,
        motor_type=args.motor_type,
    )

    if not driver.connect():
        print(f"Could not open {args.port}.")
        sys.exit(1)

    try:
        if args.both:
            test_motor(driver, 1, args.rpm, args.time)
            time.sleep(1.0)
            test_motor(driver, 2, args.rpm, args.time)
        else:
            test_motor(driver, args.motor_id, args.rpm, args.time)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
