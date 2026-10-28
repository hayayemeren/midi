#!/usr/bin/env bash
# ==============================================================================
# MIDI Mobile Manipulator Base - Raspberry Pi 4 Automated Setup Script
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}==================================================================${NC}"
echo -e "${BLUE}     MIDI Mobile Manipulator Base - Raspberry Pi Setup            ${NC}"
echo -e "${BLUE}==================================================================${NC}"

CURRENT_USER=$(whoami)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "\n${YELLOW}[1/5] Checking user and system prerequisites...${NC}"
echo "Current user: $CURRENT_USER"
echo "Project directory: $SCRIPT_DIR"

echo -e "\n${YELLOW}[2/5] Installing required system packages...${NC}"
if command -v apt-get &> /dev/null; then
    sudo apt-get update
    sudo apt-get install -y \
        python3 \
        python3-pip \
        python3-yaml \
        python3-serial \
        python3-smbus \
        i2c-tools \
        joystick
else
    echo -e "${YELLOW}apt-get not found. Skipping apt package installation.${NC}"
fi

echo -e "\n${YELLOW}[3/5] Configuring user hardware permissions...${NC}"
echo "Adding user '$CURRENT_USER' to dialout, input, and i2c groups..."
sudo usermod -a -G dialout,input,i2c "$CURRENT_USER" || true

echo -e "\n${YELLOW}[4/5] Installing Python dependencies...${NC}"
pip3 install --user -r "$SCRIPT_DIR/requirements.txt" || pip install -r "$SCRIPT_DIR/requirements.txt"

echo -e "\n${YELLOW}[5/5] Checking Raspberry Pi interfaces...${NC}"
if [ -f /boot/config.txt ] || [ -f /boot/firmware/config.txt ]; then
    echo "Ensuring I2C interface is enabled for Waveshare UPS HAT..."
    if command -v raspi-config &> /dev/null; then
        sudo raspi-config nonint do_i2c 0 || true
    fi
fi

# Optional systemd installation
echo -e "\n${BLUE}==================================================================${NC}"
echo -e "${GREEN}Software dependencies and permissions setup complete!${NC}"
echo -e "${BLUE}==================================================================${NC}"
echo ""
echo "Next steps:"
echo " 1. Make sure your user group changes take effect: log out and log back in (or run 'newgrp dialout')."
echo " 2. Pair your Logitech F710 wireless gamepad (Nano USB receiver in RPi4, switch in 'X' mode)."
echo " 3. Verify gamepad input: python3 tools/gamepad_test.py"
echo " 4. Verify motor communication: python3 tools/motor_test.py -id 1"
echo " 5. Start the controller: python3 main.py"
echo ""
read -p "Would you like to install the auto-start systemd service now? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    SERVICE_FILE="/etc/systemd/system/midi-robot.service"
    echo "Installing systemd service to $SERVICE_FILE..."
    sudo sed -e "s|User=pi|User=$CURRENT_USER|g" \
             -e "s|/home/pi/midi|$SCRIPT_DIR|g" \
             "$SCRIPT_DIR/systemd/midi-robot.service" | sudo tee "$SERVICE_FILE" > /dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable midi-robot.service
    echo -e "${GREEN}Service installed and enabled on boot!${NC}"
    echo "To start it now: sudo systemctl start midi-robot.service"
    echo "To view live logs: journalctl -u midi-robot.service -f"
fi

echo -e "\n${GREEN}Setup script finished! Enjoy your mobile manipulator!${NC}"
