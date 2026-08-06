#!/bin/bash

echo "================================================"
echo "Raspberry Pi Camera Diagnostic"
echo "================================================"
echo ""

# Check if running on Raspberry Pi
echo "1. System Information:"
if [ -f /proc/device-tree/model ]; then
    echo "   Device: $(cat /proc/device-tree/model)"
else
    echo "   Not a Raspberry Pi or device tree not available"
fi
echo ""

# Check camera detection
echo "2. Camera Detection:"
if command -v vcgencmd &> /dev/null; then
    echo "   vcgencmd detected:"
    vcgencmd get_camera
else
    echo "   ⚠️  vcgencmd not found"
fi
echo ""

# Check libcamera
echo "3. libcamera Check:"
if command -v libcamera-hello &> /dev/null; then
    echo "   ✅ libcamera-hello found"
    echo "   Testing camera (will timeout after 2 seconds)..."
    timeout 2 libcamera-hello --list-cameras 2>&1 || true
else
    echo "   ❌ libcamera not installed"
    echo "   Install with: sudo apt install -y libcamera-apps"
fi
echo ""

# Check video devices
echo "4. Available Video Devices:"
ls -la /dev/video* 2>/dev/null | awk '{print "   " $0}' || echo "   No video devices found"
echo ""

# Check which devices can capture
echo "5. Testing Video Devices:"
for i in 0 1 10 11 12 13 14 15 16 18 20 21 22 23 31; do
    if [ -e "/dev/video$i" ]; then
        echo "   /dev/video$i exists"
    fi
done
echo ""

# Check OpenCV
echo "6. OpenCV Installation:"
python3 << 'EOF'
try:
    import cv2
    print(f"   ✅ OpenCV version: {cv2.__version__}")
    
    # Check backends
    backends = []
    if cv2.CAP_V4L2:
        backends.append("V4L2")
    if cv2.CAP_GSTREAMER:
        backends.append("GStreamer")
    print(f"   Available backends: {', '.join(backends)}")
    
except ImportError:
    print("   ❌ OpenCV not installed")
EOF
echo ""

# Test camera with Python
echo "7. Quick Camera Test with OpenCV:"
python3 << 'EOF'
import cv2
import sys

print("   Testing /dev/video0...")
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if cap.isOpened():
    ret, frame = cap.read()
    if ret and frame is not None:
        print(f"   ✅ SUCCESS! Frame captured: {frame.shape}")
    else:
        print("   ⚠️  Camera opened but cannot read frames")
    cap.release()
else:
    print("   ❌ Cannot open /dev/video0")
EOF
echo ""

echo "================================================"
echo "Troubleshooting Tips:"
echo "================================================"
echo ""
echo "If camera not detected:"
echo "  1. Enable camera: sudo raspi-config -> Interface Options -> Camera"
echo "  2. Check cable connection"
echo "  3. Reboot: sudo reboot"
echo ""
echo "If libcamera missing:"
echo "  sudo apt update"
echo "  sudo apt install -y libcamera-apps"
echo ""
echo "If OpenCV can't access camera:"
echo "  sudo apt install -y python3-opencv"
echo "  pip3 install opencv-python"
echo ""
