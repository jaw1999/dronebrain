#!/bin/bash
# DroneBrain Stop Script
#
# Gracefully stops all DroneBrain processes

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Stopping DroneBrain...${NC}"

# Stop main DroneBrain process
if pgrep -f "python3 main.py" > /dev/null; then
    echo -e "${YELLOW}Stopping DroneBrain main process...${NC}"
    pkill -f "python3 main.py"
    sleep 2
    # Force kill if still running
    if pgrep -f "python3 main.py" > /dev/null; then
        pkill -9 -f "python3 main.py"
    fi
    echo -e "${GREEN}✓ DroneBrain stopped${NC}"
else
    echo -e "${YELLOW}DroneBrain main process not running${NC}"
fi

# Stop MediaMTX
if pgrep -x "mediamtx" > /dev/null; then
    echo -e "${YELLOW}Stopping MediaMTX...${NC}"
    pkill -x mediamtx
    sleep 1
    # Force kill if still running
    if pgrep -x "mediamtx" > /dev/null; then
        pkill -9 -x mediamtx
    fi
    echo -e "${GREEN}✓ MediaMTX stopped${NC}"
else
    echo -e "${YELLOW}MediaMTX not running${NC}"
fi

echo -e "${GREEN}✓ All DroneBrain services stopped${NC}"
