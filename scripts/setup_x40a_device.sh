#!/bin/bash

# Script to create persistent device name for Vstone X40A serial device
# Device: Bus 003 Device 011: ID 0403:6015 Future Technology Devices International, Ltd Bridge(I2C/SPI/UART/FIFO)
# Target: /dev/x40a_ser

set -e

UDEV_RULE_FILE="/etc/udev/rules.d/99-x40a-serial.rules"

echo "Setting up persistent device name for Vstone X40A..."

# Create udev rule
sudo tee "$UDEV_RULE_FILE" > /dev/null <<EOF
# Vstone X40A Serial Device Rule
# Future Technology Devices International, Ltd Bridge(I2C/SPI/UART/FIFO)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="x40a_ser", MODE="0666", GROUP="dialout"
EOF

echo "Created udev rule: $UDEV_RULE_FILE"

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add current user to dialout group if not already
if ! groups $USER | grep -q dialout; then
    echo "Adding user $USER to dialout group..."
    sudo usermod -a -G dialout $USER
    echo "Please logout and login again for group changes to take effect"
fi

echo "Setup complete!"
echo "Device should now be available as /dev/x40a_ser"
echo "You may need to unplug and reconnect the USB device"

# Check if device is currently connected
if lsusb | grep -q "0403:6015"; then
    echo "Device is currently connected"
    if [ -e "/dev/x40a_ser" ]; then
        echo "Symlink /dev/x40a_ser is active"
        ls -la /dev/x40a_ser
    else
        echo "Symlink not yet active - try unplugging and reconnecting the device"
    fi
else
    echo "Device not currently connected"
fi