"""DDSM (Direct Drive Smart Motor) RS485 Communication Driver.

Supports DDSM115 and DDSM210 direct drive hub motors over RS485 / UART.
Implements the 10-byte frame protocol with CRC-8/MAXIM checksum.
Synchronous Request-Response transactions prevent half-duplex RS485 bus collisions.
"""

from __future__ import annotations

import logging
import os
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any

logger = logging.getLogger("midi.ddsm")


def calculate_crc8(data: bytes) -> int:
    """Calculate CRC-8/MAXIM (Dallas/Maxim, poly 0x8C, init 0x00, reflected)."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc & 0xFF


def set_usb_latency_1ms(port: str):
    """Set FTDI / USB-serial converter latency timer from default 16ms to 1ms on Linux."""
    yol = f"/sys/bus/usb-serial/devices/{os.path.basename(port)}/latency_timer"
    try:
        with open(yol, "w") as f:
            f.write("1")
        logger.info("USB serial latency timer set to 1 ms on %s", yol)
    except Exception:
        # Requires root or udev rule; continue gracefully if not writable
        pass


@dataclass
class MotorFeedback:
    """Telemetry data returned by a DDSM motor."""
    motor_id: int = 0
    mode: int = 0
    current_raw: int = 0
    current_amps: float = 0.0
    velocity_rpm: float = 0.0
    position_raw: int = 0
    error_code: int = 0
    sensor_fault: bool = False
    over_current: bool = False
    phase_over_current: bool = False
    stall_fault: bool = False
    over_temp: bool = False
    timestamp: float = 0.0

    @classmethod
    def from_bytes(cls, data: bytes, speed_scale: float = 1.0) -> Optional[MotorFeedback]:
        """Parse 10-byte feedback frame from DDSM motor."""
        if len(data) != 10:
            return None

        # Verify CRC
        expected_crc = calculate_crc8(data[:9])
        if data[9] != expected_crc:
            return None

        motor_id = data[0]
        mode = data[1]
        raw_current = struct.unpack(">h", data[2:4])[0]
        raw_velocity = struct.unpack(">h", data[4:6])[0]
        raw_position = struct.unpack(">h", data[6:8])[0]
        err = data[8]

        current_amps = (raw_current * 8.0) / 32767.0
        velocity_rpm = raw_velocity * speed_scale

        return cls(
            motor_id=motor_id,
            mode=mode,
            current_raw=raw_current,
            current_amps=round(current_amps, 3),
            velocity_rpm=round(velocity_rpm, 2),
            position_raw=raw_position,
            error_code=err,
            sensor_fault=bool(err & 0x01),
            over_current=bool(err & 0x02),
            phase_over_current=bool(err & 0x04),
            stall_fault=bool(err & 0x08),
            over_temp=bool(err & 0x10),
            timestamp=time.time(),
        )


class DDSMDriver:
    """RS485 Bus Driver for dual DDSM motors."""

    MODE_CURRENT = 0x01
    MODE_VELOCITY = 0x02
    MODE_POSITION = 0x03

    CMD_VELOCITY_DRIVE = 0x64
    CMD_QUERY_FEEDBACK = 0x74
    CMD_SWITCH_MODE = 0xA0

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout: float = 0.02,
        dry_run: bool = False,
        motor_type: str = "ddsm115",
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.dry_run = dry_run
        self.motor_type = motor_type.lower()
        
        self.speed_scale = 10.0 if self.motor_type == "ddsm210" else 1.0

        self.serial = None
        self._lock = threading.Lock()
        self.latest_telemetry: Dict[int, MotorFeedback] = {}
        self.connected = False

    def connect(self) -> bool:
        """Open serial port connection."""
        if self.dry_run:
            logger.info("DDSMDriver running in DRY-RUN simulation mode (no serial hardware).")
            self.connected = True
            return True

        # Optimize USB latency
        set_usb_latency_1ms(self.port)

        try:
            import serial
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            self.connected = True
            logger.info("Connected to DDSM RS485 bus on %s at %d baud", self.port, self.baudrate)
            return True
        except Exception as e:
            logger.error("Failed to open serial port %s: %s", self.port, e)
            self.connected = False
            return False

    def close(self):
        """Safely close serial port."""
        with self._lock:
            if self.serial and self.serial.is_open:
                try:
                    self.serial.close()
                except Exception as e:
                    logger.debug("Error closing serial port: %s", e)
            self.connected = False
            logger.info("DDSM RS485 bus closed.")

    def set_mode(self, motor_id: int, mode: int = MODE_VELOCITY) -> bool:
        """Switch motor operating mode (Velocity Loop = 0x02)."""
        packet = bytes([motor_id & 0xFF, self.CMD_SWITCH_MODE, 0, 0, 0, 0, 0, 0, 0, mode & 0xFF])
        with self._lock:
            if self.dry_run:
                return True
            if not self.serial or not self.serial.is_open:
                return False
            try:
                self.serial.write(packet)
                self.serial.flush()
                time.sleep(0.05)
                return True
            except Exception as e:
                logger.warning("Error setting mode on motor %d: %s", motor_id, e)
                return False

    def set_velocity(
        self,
        motor_id: int,
        target_rpm: float,
        acceleration: int = 1,
        brake: bool = False,
    ) -> Optional[MotorFeedback]:
        """Send velocity command to motor and read synchronous 10-byte response.
        
        Packet layout:
          [ID, 0x64, Speed_H, Speed_L, 0, 0, Accel, Brake, 0, CRC8]
        Notice Byte 6 is Acceleration (1..255). Byte 4 and 5 are 0.
        """
        rpm = max(-330, min(330, int(round(target_rpm * self.speed_scale))))
        v = rpm & 0xFFFF
        brake_val = 0xFF if brake else 0x00
        accel_byte = max(1, min(255, acceleration))

        nine = [
            motor_id & 0xFF,
            self.CMD_VELOCITY_DRIVE,
            (v >> 8) & 0xFF,
            v & 0xFF,
            0x00,
            0x00,
            accel_byte,
            brake_val,
            0x00,
        ]
        pkt = bytes(nine) + bytes([calculate_crc8(bytes(nine))])

        if self.dry_run:
            sim_fb = MotorFeedback(
                motor_id=motor_id,
                mode=self.MODE_VELOCITY,
                velocity_rpm=float(target_rpm),
                current_amps=0.2,
                timestamp=time.time(),
            )
            self.latest_telemetry[motor_id] = sim_fb
            return sim_fb

        with self._lock:
            if not self.serial or not self.serial.is_open:
                return None
            try:
                # Flush input buffer before transmission to guarantee no stale bytes
                self.serial.reset_input_buffer()
                self.serial.write(pkt)
                self.serial.flush()

                # Read 10-byte feedback frame immediately
                r = self.serial.read(10)
                if len(r) == 10 and calculate_crc8(r[:9]) == r[9] and r[0] == motor_id:
                    fb = MotorFeedback.from_bytes(r, speed_scale=(1.0 / self.speed_scale))
                    if fb:
                        self.latest_telemetry[motor_id] = fb
                        return fb
            except Exception as e:
                logger.warning("RS485 drive error on motor %d: %s", motor_id, e)
        return None

    def emergency_brake(self, motor_id: int) -> bool:
        """Immediately engage brake on specified motor."""
        fb = self.set_velocity(motor_id=motor_id, target_rpm=0.0, acceleration=1, brake=True)
        return fb is not None

    def query_feedback(self, motor_id: int) -> Optional[MotorFeedback]:
        """Return latest feedback from motor."""
        return self.latest_telemetry.get(motor_id)

    def set_motor_id(self, new_id: int) -> bool:
        """Program new ID into connected motor (send 5 times)."""
        if not (1 <= new_id <= 254):
            raise ValueError(f"Motor ID must be between 1 and 254, got {new_id}")

        packet = bytes([0xAA, 0x55, 0x53, new_id & 0xFF, 0, 0, 0, 0, 0, 0])
        if self.dry_run:
            return True

        with self._lock:
            if not self.serial or not self.serial.is_open:
                return False
            try:
                for _ in range(5):
                    self.serial.write(packet)
                    self.serial.flush()
                    time.sleep(0.05)
                return True
            except Exception as e:
                logger.error("Failed to send set ID command: %s", e)
                return False
