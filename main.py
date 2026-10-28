#!/usr/bin/env python3
"""MIDI - Mobile Manipulator Dual DDSM Motor & Gamepad Control System.

Main application entry point for Raspberry Pi.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import yaml

# Add project root to sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.robot import MidiRobot


def setup_logging(verbose: bool = False):
    """Configure formatted console logging."""
    level = logging.DEBUG if verbose else logging.INFO
    log_format = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    logging.basicConfig(level=level, format=log_format, datefmt="%H:%M:%S")


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file."""
    if not os.path.exists(config_path):
        logging.error("Configuration file '%s' not found!", config_path)
        sys.exit(1)

    with open(config_path, "r", encoding="utf-8") as f:
        try:
            return yaml.safe_load(f) or {}
        except Exception as e:
            logging.error("Failed to parse YAML configuration: %s", e)
            sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="MIDI Robot: Dual DDSM Motor & Logitech F710 Controller"
    )
    parser.add_argument(
        "-c", "--config",
        default="config.yaml",
        help="Path to YAML configuration file (default: config.yaml)"
    )
    parser.add_argument(
        "-p", "--port",
        default=None,
        help="Override RS485 serial port (e.g. /dev/ttyUSB0)"
    )
    parser.add_argument(
        "-d", "--dry-run",
        action="store_true",
        help="Run in simulation mode without hardware RS485"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable debug logging output"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    setup_logging(args.verbose)

    logging.info("Starting MIDI Mobile Manipulator Base Controller...")
    config = load_config(args.config)

    # Command line overrides
    if args.port:
        config.setdefault("serial", {})["port"] = args.port
    if args.dry_run:
        config["dry_run"] = True

    robot = MidiRobot(config)
    try:
        robot.start()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received.")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
