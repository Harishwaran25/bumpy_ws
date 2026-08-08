                           
#!/bin/bash

echo "➡️ Configuring AMCL..."
ros2 lifecycle set /amcl configure
sleep 2

echo "➡️ Activating AMCL..."
ros2 lifecycle set /amcl activate
sleep 2

echo "✅ AMCL activation commands executed"
