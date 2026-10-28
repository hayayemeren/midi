"""DDSM (Direct Drive Smart Motor) RS485 Communication Driver.

Supports DDSM115 and DDSM210 direct drive hub motors over RS485 / UART.
Implements the 10-byte frame protocol with CRC-8/MAXIM checksum.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any

logger = logging.getLogger("midi.ddsm")


def calculate_crc8(data: bytes) -> int:
    """Calculate CRC-8/MAXIM (Dallas/Maxim, poly 0x8C, init 0x00, reflected).
    
    Args:
        data: Byte payload to checksum.
        
    Returns:
        1-byte CRC checksum as integer (0-255).
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF


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
        """Parse 10-byte feedback frame from DDSM motor.
        
        Frame layout:
          Byte 0: Motor ID
          Byte 1: Mode (0x01: Current, 0x02: Velocity, 0x03: Position)
          Byte 2-3: Current (int16, Big-Endian)
          Byte 4-5: Velocity (int16, Big-Endian)
          Byte 6-7: Position (int16/uint16, Big-Endian)
          Byte 8: Error code bitmask
          Byte 9: CRC-8
        """
        if len(data) != 10:
            return None

        # Verify CRC
        expected_crc = calculate_crc8(data[:9])
        if data[9] != expected_crc:
            logger.warning(
                "CRC mismatch for motor %d: expected 0x%02X, got 0x%02X",
                data[0], expected_crc, data[9]
            )
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
        timeout: float = 0.05,
        dry_run: bool = False,
        motor_type: str = "ddsm115",
    ):
        """Initialize DDSM RS485 driver.
        
        Args:
            port: Serial device path (e.g., /dev/ttyUSB0).
            baudrate: Baudrate (default 115200).
            timeout: Read timeout in seconds.
            dry_run: If True, simulate communication without opening hardware serial port.
            motor_type: 'ddsm115' (1 RPM increments) or 'ddsm210' (0.1 RPM increments).
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.dry_run = dry_run
        self.motor_type = motor_type.lower()
        
        # Scaling factor: DDSM115 cmd is 1 rpm, DDSM210 is 0.1 rpm
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
        """Switch motor operating mode.
        
        Packet: [ID, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, Mode]
        (Mode frame has no CRC, the 10th byte is the mode byte).
        """
        packet = bytearray(10)
        packet[0] = motor_id & 0xFF
        packet[1] = self.CMD_SWITCH_MODE
        packet[9] = mode & 0xFF

        return self._send_packet(bytes(packet), expect_response=False)

    def set_velocity(
        self,
        motor_id: int,
        target_rpm: float,
        acceleration: int = 50,
        brake: bool = False,
    ) -> bool:
        """Send velocity command to motor.
        
        Args:
            motor_id: ID of target motor (e.g., 1 or 2).
            target_rpm: Target rotational speed in RPM (signed).
            acceleration: Acceleration ramp time (0-255, 0.1 ms/rev).
            brake: If True, engages electric brake (speed forced to 0).
        """
        # Apply scaling based on motor type
        raw_speed = int(round(target_rpm * self.speed_scale))
        # Clamp to int16 limits
        raw_speed = max(-32767, min(32767, raw_speed))

        accel_val = max(0, min(65535, acceleration))
        brake_val = 0xFF if brake else 0x00
        if brake:
            raw_speed = 0

        packet = bytearray(10)
        packet[0] = motor_id & 0xFF
        packet[1] = self.CMD_VELOCITY_DRIVE
        struct.pack_into(">h", packet, 2, raw_speed)
        struct.pack_into(">H", packet, 4, accel_val)
        packet[6] = 0x00
        packet[7] = brake_val
        packet[8] = 0x00
        packet[9] = calculate_crc8(bytes(packet[:9]))

        return self._send_packet(bytes(packet), expect_response=False)

    def emergency_brake(self, motor_id: int) -> bool:
        """Immediately engage brake on specified motor."""
        return self.set_velocity(motor_id=motor_id, target_rpm=0.0, brake=True)

    def query_feedback(self, motor_id: int) -> Optional[MotorFeedback]:
        """Query motor for current telemetry status.
        
        Packet: [ID, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CRC]
        """
        packet = bytearray(10)
        packet[0] = motor_id & 0xFF
        packet[1] = self.CMD_QUERY_FEEDBACK
        packet[9] = calculate_crc8(bytes(packet[:9]))

        if self.dry_run:
            sim_fb = MotorFeedback(
                motor_id=motor_id,
                mode=self.MODE_VELOCITY,
                velocity_rpm=0.0,
                current_amps=0.2,
                timestamp=time.time(),
            )
            self.latest_telemetry[motor_id] = sim_fb
            return sim_fb

        with self._lock:
            if not self.serial or not self.serial.is_open:
                return None
            try:
                self.serial.reset_input_buffer()
                self.serial.write(packet)
                self.serial.flush()
                
                resp = self.serial.read(10)
                if len(resp) == 10:
                    fb = MotorFeedback.from_bytes(resp, speed_scale=(1.0 / self.speed_scale))
                    if fb:
                        self.latest_telemetry[motor_id] = fb
                        return fb
                else:
                    logger.debug("Motor %d query timeout: read %d bytes", motor_id, len(resp))
            except Exception as e:
                logger.warning("Error querying motor %d: %s", motor_id, e)
        return None

    def set_motor_id(self, new_id: int) -> bool:
        """Program new ID into connected motor.
        
        CRITICAL: ONLY ONE MOTOR MUST BE CONNECTED TO THE RS485 BUS!
        Packet: [0xAA, 0x55, 0x53, new_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        Sent 5 times consecutively per manufacturer specification.
        """
        if not (1 <= new_id <= 254):
            raise ValueError(f"Motor ID must be between 1 and 254, got {new_id}")

        packet = bytearray(10)
        packet[0] = 0xAA
        packet[1] = 0x55
        packet[2] = 0x53
        packet[3] = new_id & 0xFF

        logger.info("Broadcasting ID change to %d (sending 5 frames)...", new_id)
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

    def _send_packet(self, packet: bytes, expect_response: bool = False) -> bool:
        """Internal helper to write packet to RS485 bus."""
        if self.dry_run:
            return True

        with self._lock:
            if not self.serial or not self.serial.is_open:
                return False
            try:
                self.serial.write(packet)
                self.serial.flush()
                return True
            except Exception as e:
                logger.warning("RS485 write error: %s", e)
                return False
