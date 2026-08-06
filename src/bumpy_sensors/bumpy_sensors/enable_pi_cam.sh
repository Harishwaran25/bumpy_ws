#!/bin/bash

echo "================================================"
echo "Raspberry Pi Camera Setup for Ubuntu Server 22.04"
echo "================================================"
echo ""

echo "STEP 1: Check Current Camera Status"
echo "===================================="
echo "Checking if camera is detected..."
vcgencmd get_camera 2>/dev/null || echo "vcgencmd not available"
echo ""

echo "STEP 2: Enable Camera in Boot Configuration"
echo "============================================"
echo "Editing /boot/firmware/config.txt to enable camera..."
echo ""

# Backup config.txt
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.backup
echo "✅ Backed up config.txt to config.txt.backup"
echo ""

# Check if camera is already enabled
if grep -q "^start_x=1" /boot/firmware/config.txt; then
    echo "✅ Camera already enabled in config.txt"
else
    echo "Adding camera configuration..."
    
    # Add camera settings
    sudo bash -c 'cat >> /boot/firmware/config.txt << EOF

# Enable Raspberry Pi Camera
start_x=1
gpu_mem=128
EOF'
    echo "✅ Camera configuration added"
fi
echo ""

echo "STEP 3: Load Camera Kernel Modules"
echo "==================================="
echo "Loading bcm2835-v4l2 module..."

# Load the module
sudo modprobe bcm2835-v4l2

# Add to /etc/modules for auto-load on boot
if ! grep -q "bcm2835-v4l2" /etc/modules; then
    echo "bcm2835-v4l2" | sudo tee -a /etc/modules
    echo "✅ Added bcm2835-v4l2 to /etc/modules for auto-load"
else
    echo "✅ bcm2835-v4l2 already in /etc/modules"
fi
echo ""

echo "STEP 4: Check Video Devices"
echo "============================"
echo "Available video devices:"
ls -l /dev/video* 2>/dev/null || echo "No video devices found yet"
echo ""

echo "================================================"
echo "SETUP COMPLETE!"
echo "================================================"
echo ""
echo "IMPORTANT: You MUST REBOOT for changes to take effect!"
echo ""
echo "After reboot, verify with:"
echo "  1. Check devices: ls -l /dev/video*"
echo "  2. Test camera: python3 find_working_camera.py"
echo ""
echo "To reboot now, run:"
echo "  sudo reboot"
echo ""
