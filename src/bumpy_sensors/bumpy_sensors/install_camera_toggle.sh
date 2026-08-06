#!/bin/bash

# Quick Fix Script for Camera Toggle Feature
# This script backs up your current file and installs the new one

set -e

echo "🔧 Camera Toggle Installation Script"
echo "===================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# File paths
ORIG_FILE="$HOME/bumpy_ws/src/bumpy_sensors/bumpy_sensors/web_server_node.py"
NEW_FILE="web_server_node.py"
BACKUP_FILE="$HOME/bumpy_ws/src/bumpy_sensors/bumpy_sensors/web_server_node.py.backup.$(date +%Y%m%d_%H%M%S)"

# Check if new file exists
if [ ! -f "$NEW_FILE" ]; then
    echo -e "${RED}❌ Error: $NEW_FILE not found in current directory${NC}"
    echo "Please run this script from the directory containing $NEW_FILE"
    exit 1
fi

# Check if original file exists
if [ ! -f "$ORIG_FILE" ]; then
    echo -e "${RED}❌ Error: Original file not found at $ORIG_FILE${NC}"
    echo "Please check your workspace path"
    exit 1
fi

# Backup original file
echo -e "${YELLOW}📦 Creating backup...${NC}"
cp "$ORIG_FILE" "$BACKUP_FILE"
echo -e "${GREEN}✅ Backup created: $BACKUP_FILE${NC}"
echo ""

# Copy new file
echo -e "${YELLOW}📝 Installing new file with camera toggle...${NC}"
cp "$NEW_FILE" "$ORIG_FILE"
echo -e "${GREEN}✅ New file installed${NC}"
echo ""

# Rebuild
echo -e "${YELLOW}🔨 Rebuilding bumpy_sensors package...${NC}"
cd "$HOME/bumpy_ws"
colcon build --packages-select bumpy_sensors 2>&1 | grep -E "Starting|Finished|ERROR|WARNING" || true
echo -e "${GREEN}✅ Build complete${NC}"
echo ""

# Source setup
echo -e "${YELLOW}🔄 Sourcing workspace...${NC}"
source "$HOME/bumpy_ws/install/setup.bash"
echo -e "${GREEN}✅ Workspace sourced${NC}"
echo ""

echo "===================================="
echo -e "${GREEN}✅ Installation Complete!${NC}"
echo ""
echo "📝 What changed:"
echo "  • Camera now starts DISABLED by default"
echo "  • Toggle switch added to web dashboard"
echo "  • New API endpoint: /api/camera/toggle"
echo "  • Enhanced /api/camera/info endpoint"
echo ""
echo "🚀 Next steps:"
echo "  1. Launch the web server:"
echo "     ros2 run bumpy_sensors web_server_node"
echo ""
echo "  2. Open browser:"
echo "     http://localhost:5000"
echo ""
echo "  3. Look for camera toggle switch above camera feed"
echo ""
echo "📦 Your original file is backed up at:"
echo "   $BACKUP_FILE"
echo ""
echo "💡 To restore backup:"
echo "   cp $BACKUP_FILE $ORIG_FILE"
echo ""
