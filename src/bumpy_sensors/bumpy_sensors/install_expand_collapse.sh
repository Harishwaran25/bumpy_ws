#!/bin/bash

# Installation Script for Expand/Collapse Dashboard
# Adds collapsible sections to all dashboard panels

set -e

echo "📦 Installing Expand/Collapse Feature"
echo "====================================="
echo ""

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ORIG_FILE="$HOME/bumpy_ws/src/bumpy_sensors/bumpy_sensors/web_server_node.py"
NEW_FILE="web_server_node.py"
BACKUP_FILE="$ORIG_FILE.backup.$(date +%Y%m%d_%H%M%S)"

# Check files
if [ ! -f "$NEW_FILE" ]; then
    echo -e "${YELLOW}❌ Error: $NEW_FILE not found${NC}"
    exit 1
fi

if [ ! -f "$ORIG_FILE" ]; then
    echo -e "${YELLOW}❌ Error: Original file not found${NC}"
    exit 1
fi

# Backup
echo -e "${BLUE}📦 Creating backup...${NC}"
cp "$ORIG_FILE" "$BACKUP_FILE"
echo -e "${GREEN}✅ Backup: $BACKUP_FILE${NC}"
echo ""

# Install
echo -e "${BLUE}📝 Installing new version...${NC}"
cp "$NEW_FILE" "$ORIG_FILE"
echo -e "${GREEN}✅ Installed${NC}"
echo ""

# Rebuild
echo -e "${BLUE}🔨 Rebuilding package...${NC}"
cd "$HOME/bumpy_ws"
colcon build --packages-select bumpy_sensors 2>&1 | grep -E "Starting|Finished|ERROR" || true
echo -e "${GREEN}✅ Build complete${NC}"
echo ""

# Source
echo -e "${BLUE}🔄 Sourcing workspace...${NC}"
source "$HOME/bumpy_ws/install/setup.bash"
echo -e "${GREEN}✅ Ready!${NC}"
echo ""

echo "====================================="
echo -e "${GREEN}✅ Installation Complete!${NC}"
echo ""
echo "✨ NEW FEATURES:"
echo "  • All sections now collapsible"
echo "  • Click section headers to expand/collapse"
echo "  • State persists across page reloads"
echo "  • Smooth animations"
echo ""
echo "📦 COLLAPSIBLE SECTIONS:"
echo "  • 🎮 Manual Control"
echo "  • 📊 Robot Status"
echo "  • 🗺️ SLAM Mapping"
echo "  • 📍 Waypoint Navigation"
echo "  • 📹 Camera Feed"
echo ""
echo "🚀 Start the server:"
echo "   ros2 run bumpy_sensors web_server_node"
echo ""
echo "🌐 Open browser:"
echo "   http://localhost:5000"
echo ""
echo "💡 TIP: Click any section header (▼) to collapse it!"
echo ""
