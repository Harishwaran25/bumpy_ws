#!/usr/bin/env python3

import cv2
import sys

video_devices = [0, 1, 10, 11, 12, 13, 14, 15, 16, 18, 20, 21, 22, 23, 31]

print("=" * 60)
print("Testing all video devices on Raspberry Pi")
print("=" * 60)
print()

working_devices = []

for device_num in video_devices:
    print(f"Testing /dev/video{device_num}...")
    
    # Try V4L2
    try:
        cap = cv2.VideoCapture(device_num, cv2.CAP_V4L2)
        
        if cap.isOpened():
            # Try to set some common formats
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # Try to read a frame
            ret, frame = cap.read()
            
            if ret and frame is not None:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = int(cap.get(cv2.CAP_PROP_FPS))
                
                print(f"  ✅ SUCCESS!")
                print(f"     Resolution: {width}x{height}")
                print(f"     FPS: {fps}")
                print(f"     Frame shape: {frame.shape}")
                
                working_devices.append({
                    'device': device_num,
                    'width': width,
                    'height': height,
                    'fps': fps
                })
            else:
                print(f"  ⚠️  Device opens but cannot read frames")
            
            cap.release()
        else:
            print(f"  ❌ Cannot open device")
    
    except Exception as e:
        print(f"  ❌ Error: {e}")
    
    print()

print("=" * 60)
print("SUMMARY")
print("=" * 60)

if working_devices:
    print(f"\n✅ Found {len(working_devices)} working camera(s):\n")
    for dev in working_devices:
        print(f"   /dev/video{dev['device']}: {dev['width']}x{dev['height']} @ {dev['fps']}fps")
    
    print("\n📝 Recommended device: /dev/video" + str(working_devices[0]['device']))
else:
    print("\n❌ No working cameras found!")
    print("\nPossible reasons:")
    print("  1. Camera is not enabled in raspi-config")
    print("  2. Camera cable is not properly connected")
    print("  3. Camera is not compatible")
    print("  4. Need to install camera drivers")
    print("\nRun these commands:")
    print("  sudo raspi-config  # Enable camera interface")
    print("  sudo reboot")
    print("  vcgencmd get_camera  # Should show: supported=1 detected=1")

print()
