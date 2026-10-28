# MIDI Mobile Manipulator Base Controller

Control software for a mobile manipulator base powered by dual **DDSM direct-drive hub motors** (DDSM115 / DDSM210), controlled wirelessly via a **Logitech F710 Gamepad**, hosted on a **Raspberry Pi 4** with a **Waveshare UPS HAT** and an **18V Makita power supply**.

---

## System Architecture

```
                  ┌────────────────────────────────────────┐
                  │       Makita 18V LXT Battery           │
                  └───────────────┬────────────────────────┘
                                  │ (Main 18V DC Bus)
                 ┌────────────────┴────────────────┐
                 │                                 │
                 ▼                                 ▼
    ┌──────────────────────────┐      ┌──────────────────────────┐
    │  DDSM Motors (Left/Right)│      │  DC-DC Step-Down (5V/5A) │
    │  Direct 18V Drive Power  │      │  or UPS HAT Power In     │
    └────────────▲─────────────┘      └────────────┬─────────────┘
                 │ RS485                           │ 5V Power
                 │ (A+, B-, GND)                   ▼
    ┌────────────┴─────────────┐      ┌──────────────────────────┐
    │  USB-to-RS485 Dongle     │◄─────┤   Raspberry Pi 4 Model B │
    └──────────────────────────┘      │   with Waveshare UPS HAT │
                                      └────────────▲─────────────┘
                                                   │ 2.4GHz RF / USB
                                      ┌────────────┴─────────────┐
                                      │ Logitech F710 Controller │
                                      │ (Mode Switch: 'X')       │
                                      └──────────────────────────┘
```

---

## Hardware Specifications

| Component | Role | Details / Connection |
| :--- | :--- | :--- |
| **DDSM Motors** | Left & Right Drive Hubs | Waveshare DDSM115 / DDSM210 (18V rated, RS485 115200 baud) |
| **Logitech F710** | Wireless Teleoperation | Wireless gamepad with USB nano receiver (set switch to **'X'** mode) |
| **Raspberry Pi 4** | Onboard Computer | Runs kinematics, watchdog, and motor communication |
| **Waveshare UPS HAT**| Power Backup & Telemetry | I2C power monitoring (INA219) on address `0x41` or `0x42` |
| **Makita 18V** | Primary Power Supply | 18V nominal (15.0V - 20.5V) Lithium-ion pack |
| **RS485 Adapter** | Motor Serial Interface | USB-to-RS485 dongle (`/dev/ttyUSB0`) |

---

## Logitech F710 Gamepad Control Mapping

> [!TIP]
> Ensure the hardware slide switch on the front top edge of the Logitech F710 is set to **`X`** (XInput mode) for native Linux driver support without extra software.

| Control Input | Function | Description |
| :--- | :--- | :--- |
| **Left Stick (Vertical)** | **Drive Forward / Backward ($v$)** | Push Forward to drive forward, Pull Back to reverse |
| **Right Stick (Horizontal)**| **Steer Left / Right ($\omega$)** | Push Left to turn counter-clockwise, Right to turn clockwise |
| **Left Bumper (LB)** | **Speed UP (+)** | Increases speed level by 1 gear (20% → 40% → 60% → 80% → 100%) |
| **Right Bumper (RB)** | **Speed DOWN (-)** | Decreases speed level by 1 gear (100% → 80% → 60% → 40% → 20%) |
| **Button B / BACK** | **EMERGENCY STOP** | Instantly halts both motors and engages electric brake |
| **Button A / START**| **Clear E-Stop** | Clears the emergency stop latch and re-arms the drive base |
| **D-Pad (Up/Down/Left/Right)** | **Fine Inching** | Precision low-speed pulse nudging for docking maneuvers |

---

## Step-by-Step Setup on Raspberry Pi

### 1. Clone Repository on Raspberry Pi
Open a terminal on your Raspberry Pi and clone this repository:
```bash
git clone https://github.com/hayayemeren/midi.git
cd midi
```

### 2. Run Automated Setup Script
Run the installation script to configure system packages, permissions, and Python dependencies:
```bash
chmod +x setup.sh
./setup.sh
```

The script will:
* Install required system packages (`python3-pip`, `i2c-tools`, `python3-serial`, `python3-yaml`).
* Grant your user permission to access `/dev/ttyUSB*` (dialout), `/dev/input/js*` (input), and I2C without `sudo`.
* Enable the Raspberry Pi I2C interface.
* Optionally install and enable the automatic systemd background service.

> [!IMPORTANT]
> If this is your first time setting permissions, log out and back in or run `newgrp dialout` for group permissions to take effect.

---

## Motor ID Configuration (First Time Only)

DDSM motors from the factory often both ship with default **ID 1**. Because RS485 is a shared multi-drop bus, each motor on the robot must have a unique ID:
* **Left Motor**: ID `1`
* **Right Motor**: ID `2`

> [!CAUTION]
> **Connect ONLY ONE motor to the RS485 bus when programming the ID!**
> If both motors are plugged in, both will be reprogrammed to the same ID.

To assign ID `2` to the right motor:
1. Disconnect the left motor's RS485 cable.
2. Plug only the right motor into the RS485 adapter.
3. Run:
   ```bash
   python3 tools/set_motor_id.py --new-id 2
   ```
4. Power-cycle the motor (unplug and replug the Makita 18V power lead).
5. Reconnect both motors in parallel to the RS485 bus (A+ to A+, B- to B-, GND to GND).

---

## Testing & Diagnostics

Before driving, run the built-in diagnostic tools:

### 1. Verify Gamepad
Plug the Logitech F710 USB nano receiver into the Raspberry Pi 4 and run:
```bash
python3 tools/gamepad_test.py
```
Move the sticks and press buttons to confirm inputs and communication.

### 2. Test Motor Spinning
Test that each motor spins safely at a low test speed (30 RPM):
```bash
# Test Left Motor (ID 1)
python3 tools/motor_test.py -id 1 -r 30 -t 3

# Test Right Motor (ID 2)
python3 tools/motor_test.py -id 2 -r 30 -t 3

# Test Both Motors Consecutively
python3 tools/motor_test.py --both -r 25
```

### 3. Verify Battery Telemetry
Check the Waveshare UPS HAT I2C communication:
```bash
python3 tools/battery_status.py
```

---

## Running the Robot Controller

### Manual Run
To start the main robot controller:
```bash
python3 main.py
```

Options:
* `--dry-run` (`-d`): Run in simulation mode without serial hardware connected.
* `--verbose` (`-v`): Enable detailed debug output.
* `--port /dev/ttyUSB0` (`-p`): Override serial port path.

### Auto-start on Boot (systemd service)
To run the robot as a background daemon whenever the Raspberry Pi boots:

```bash
# Start service
sudo systemctl start midi-robot.service

# View live log output
journalctl -u midi-robot.service -f

# Check service status
sudo systemctl status midi-robot.service
```

---

## Configuration Reference (`config.yaml`)

Edit [`config.yaml`](file:///home/maksimgorki/midi/config.yaml) to tune your robot's parameters:

```yaml
serial:
  port: "/dev/ttyUSB0"          # RS485 adapter path
  baudrate: 115200

motors:
  type: "ddsm115"               # ddsm115 or ddsm210
  left_id: 1
  right_id: 2
  invert_left: false
  invert_right: true            # Inverted for mirrored chassis mount

robot:
  wheel_radius: 0.0575          # meters (115mm diameter / 2)
  wheel_base: 0.350             # meters (track width)
  max_linear_speed: 0.80        # m/s (~2.88 km/h)
  max_angular_speed: 2.0        # rad/s
  linear_accel_limit: 1.5       # m/s^2 (smooth ramp to prevent tipping)
  angular_accel_limit: 3.0      # rad/s^2

gamepad:
  deadzone: 0.10                # Stick deadband
  expo: 1.4                     # Curve factor for fine centering
  require_deadman: false        # Require holding LB to drive
```

---

## Safety Features

* **Communication Watchdog**: If gamepad wireless signal is lost for >0.5 seconds, the robot commands an instant stop (`0 RPM`).
* **Emergency Stop Latch**: Pressing **B** or **BACK** locks the system into E-Stop mode with motor brakes engaged.
* **Slew-Rate Acceleration Limiting**: Smooth velocity ramping prevents sudden torque shocks that could tip the manipulator or over-current the Makita battery.
* **Graceful Termination**: Captures `SIGINT` / `SIGTERM` to safely brake both motors before exit.
