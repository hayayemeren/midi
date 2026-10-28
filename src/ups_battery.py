"""Waveshare UPS HAT & Makita 18V Battery Monitor for Raspberry Pi.

Interfaces with the on-board INA219 power monitoring IC over I2C to read
voltage, current, power, and state of charge (SoC).
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger("midi.battery")


@dataclass
class BatteryTelemetry:
    """Power and battery status snapshot."""
    bus_voltage_v: float = 0.0
    shunt_voltage_mv: float = 0.0
    current_ma: float = 0.0
    power_mw: float = 0.0
    percentage: float = 0.0
    is_charging: bool = False
    connected: bool = False
    timestamp: float = 0.0


class WaveshareUPSMonitor:
    """I2C Driver for Waveshare UPS HAT (INA219 based)."""

    # INA219 Registers
    REG_CONFIG = 0x00
    REG_SHUNTVOLTAGE = 0x01
    REG_BUSVOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05

    def __init__(
        self,
        i2c_bus: int = 1,
        i2c_address: int = 0x41,  # Waveshare UPS HAT B default is 0x41 or 0x42
        battery_type: str = "ups_2s",  # "ups_2s" (7.4V/8.4V) or "makita_18v" (18V/21V)
        enabled: bool = True,
    ):
        self.i2c_bus_num = i2c_bus
        self.i2c_address = i2c_address
        self.battery_type = battery_type
        self.enabled = enabled

        self._bus = None
        self._cal_value = 4096
        self._current_lsb = 0.1  # 0.1 mA per bit
        self._power_lsb = 0.002   # 2 mW per bit
        self.connected = False

    def init(self) -> bool:
        """Initialize I2C bus and calibrate INA219 sensor."""
        if not self.enabled:
            logger.info("UPS Battery monitoring is disabled in config.")
            return False

        try:
            import smbus2
            self._bus = smbus2.SMBus(self.i2c_bus_num)
        except ImportError:
            try:
                import smbus
                self._bus = smbus.SMBus(self.i2c_bus_num)
            except ImportError:
                logger.debug("smbus/smbus2 not installed. Battery monitoring disabled.")
                return False
        except Exception as e:
            logger.warning("Could not open I2C bus %d: %s", self.i2c_bus_num, e)
            return False

        try:
            # Configure INA219: 32V range, +/-320mV shunt, 12-bit ADC, continuous
            # Config = 0x399F (32V, 320mV, 12-bit, continuous)
            self._write_register(self.REG_CONFIG, 0x399F)
            self._write_register(self.REG_CALIBRATION, self._cal_value)
            self.connected = True
            logger.info("Waveshare UPS HAT detected on I2C address 0x%02X", self.i2c_address)
            return True
        except Exception as e:
            logger.warning("INA219 not responding on 0x%02X: %s", self.i2c_address, e)
            self.connected = False
            return False

    def read(self) -> BatteryTelemetry:
        """Read voltage, current, and compute battery percentage."""
        if not self.connected or not self._bus:
            return BatteryTelemetry(connected=False, timestamp=time.time())

        try:
            # Read Bus Voltage (Shifted right by 3 bits, 4mV per LSB)
            raw_bus = self._read_register(self.REG_BUSVOLTAGE)
            bus_voltage = (raw_bus >> 3) * 0.004

            # Read Shunt Voltage (10uV per LSB)
            raw_shunt = self._read_register_signed(self.REG_SHUNTVOLTAGE)
            shunt_voltage_mv = raw_shunt * 0.01

            # Read Current
            # Re-write calibration to ensure valid current reading
            self._write_register(self.REG_CALIBRATION, self._cal_value)
            raw_current = self._read_register_signed(self.REG_CURRENT)
            current_ma = raw_current * self._current_lsb

            # Read Power
            raw_power = self._read_register(self.REG_POWER)
            power_mw = raw_power * (self._power_lsb * 1000)

            # Compute state of charge percentage
            pct = self._calculate_percentage(bus_voltage)

            return BatteryTelemetry(
                bus_voltage_v=round(bus_voltage, 2),
                shunt_voltage_mv=round(shunt_voltage_mv, 2),
                current_ma=round(current_ma, 1),
                power_mw=round(power_mw, 1),
                percentage=round(pct, 1),
                is_charging=(current_ma < 0),
                connected=True,
                timestamp=time.time(),
            )
        except Exception as e:
            logger.debug("Error reading INA219 battery telemetry: %s", e)
            return BatteryTelemetry(connected=False, timestamp=time.time())

    def _calculate_percentage(self, voltage: float) -> float:
        """Estimate percentage from discharge curve."""
        if self.battery_type == "makita_18v":
            # 5S Li-ion Makita Pack: 15.0V (0%) to 20.5V (100%)
            min_v, max_v = 15.0, 20.5
        else:
            # 2S Li-ion UPS HAT Pack: 6.0V (0%) to 8.4V (100%)
            min_v, max_v = 6.2, 8.35

        pct = ((voltage - min_v) / (max_v - min_v)) * 100.0
        return max(0.0, min(100.0, pct))

    def _write_register(self, register: int, value: int):
        high_byte = (value >> 8) & 0xFF
        low_byte = value & 0xFF
        self._bus.write_i2c_block_data(self.i2c_address, register, [high_byte, low_byte])

    def _read_register(self, register: int) -> int:
        data = self._bus.read_i2c_block_data(self.i2c_address, register, 2)
        return (data[0] << 8) | data[1]

    def _read_register_signed(self, register: int) -> int:
        val = self._read_register(register)
        if val > 32767:
            val -= 65536
        return val
