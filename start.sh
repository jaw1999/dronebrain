#!/bin/bash
# DroneBrain Startup Script
#
# Usage:
#   ./start.sh              # Normal mode
#   ./start.sh --dev        # Development mode (debug logging)
#   ./start.sh --help       # Show help

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo -e "${BLUE}"
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║                                                           ║"
echo "║                      DRONEBRAIN                           ║"
echo "║                    Startup Script                         ║"
echo "║                                                           ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check Python version
echo -e "${YELLOW}Checking Python version...${NC}"
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Error: python3 not found${NC}"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo -e "${GREEN}✓ Python $PYTHON_VERSION${NC}"

# Check if venv exists
if [ ! -d "venv" ]; then
    echo -e "${YELLOW}Virtual environment not found. Creating...${NC}"
    python3 -m venv venv
    echo -e "${GREEN}✓ Virtual environment created${NC}"
fi

# Activate virtual environment
echo -e "${YELLOW}Activating virtual environment...${NC}"
source venv/bin/activate
echo -e "${GREEN}✓ Virtual environment activated${NC}"

# Check if dependencies are installed
if [ ! -f "venv/.dependencies_installed" ]; then
    echo -e "${YELLOW}Installing dependencies...${NC}"
    pip install --upgrade pip
    pip install -r requirements.txt
    touch venv/.dependencies_installed
    echo -e "${GREEN}✓ Dependencies installed${NC}"
else
    echo -e "${GREEN}✓ Dependencies already installed${NC}"
fi

# Check if config exists
if [ ! -f "config/config.yaml" ]; then
    echo -e "${RED}Error: config/config.yaml not found${NC}"
    echo -e "${YELLOW}Copying example config...${NC}"
    if [ -f "config/config.example.yaml" ]; then
        cp config/config.example.yaml config/config.yaml
        echo -e "${GREEN}✓ Config created from example${NC}"
        echo -e "${YELLOW}Please edit config/config.yaml with your settings${NC}"
        exit 1
    else
        echo -e "${RED}Error: config.example.yaml not found${NC}"
        exit 1
    fi
fi

# Create logs directory
mkdir -p logs

# Check hardware connections (optional)
echo -e "${YELLOW}Checking hardware connections...${NC}"

# Ping drone
if ping -c 1 -W 1 192.168.144.102 &> /dev/null; then
    echo -e "${GREEN}✓ Drone reachable at 192.168.144.102${NC}"
else
    echo -e "${YELLOW}⚠ Drone not reachable at 192.168.144.102${NC}"
fi

# Ping camera/gimbal
if ping -c 1 -W 1 192.168.144.25 &> /dev/null; then
    echo -e "${GREEN}✓ Siyi camera reachable at 192.168.144.25${NC}"
else
    echo -e "${YELLOW}⚠ Siyi camera not reachable at 192.168.144.25${NC}"
fi

# Start MediaMTX streaming server
echo -e "${YELLOW}Starting MediaMTX streaming server...${NC}"
if command -v mediamtx &> /dev/null; then
    # Check if MediaMTX config exists
    if [ ! -f "config/mediamtx.yml" ]; then
        echo -e "${RED}Error: config/mediamtx.yml not found${NC}"
        exit 1
    fi

    # Check if MediaMTX is already running
    if pgrep -x "mediamtx" > /dev/null; then
        echo -e "${YELLOW}⚠ MediaMTX already running, stopping old instance...${NC}"
        pkill -x mediamtx
        sleep 1
    fi

    # Start MediaMTX in background
    mediamtx config/mediamtx.yml > logs/mediamtx.log 2>&1 &
    MEDIAMTX_PID=$!
    sleep 1

    # Verify it started
    if kill -0 $MEDIAMTX_PID 2>/dev/null; then
        echo -e "${GREEN}✓ MediaMTX started (PID: $MEDIAMTX_PID)${NC}"
        echo -e "${GREEN}  WebRTC: http://localhost:8889${NC}"
        echo -e "${GREEN}  RTSP: rtsp://localhost:8554${NC}"
        echo -e "${GREEN}  HLS: http://localhost:8888${NC}"
    else
        echo -e "${RED}✗ MediaMTX failed to start${NC}"
        exit 1
    fi
else
    echo -e "${YELLOW}⚠ MediaMTX not found in PATH, video streaming will not work${NC}"
    MEDIAMTX_PID=""
fi

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down DroneBrain...${NC}"

    # Stop MediaMTX if we started it
    if [ ! -z "$MEDIAMTX_PID" ] && kill -0 $MEDIAMTX_PID 2>/dev/null; then
        echo -e "${YELLOW}Stopping MediaMTX...${NC}"
        kill $MEDIAMTX_PID
        sleep 1
        # Force kill if still running
        if kill -0 $MEDIAMTX_PID 2>/dev/null; then
            kill -9 $MEDIAMTX_PID 2>/dev/null
        fi
        echo -e "${GREEN}✓ MediaMTX stopped${NC}"
    fi

    echo -e "${GREEN}✓ Shutdown complete${NC}"
    deactivate
}

# Register cleanup on exit
trap cleanup EXIT INT TERM

echo ""
echo -e "${BLUE}Starting DroneBrain...${NC}"
echo -e "${YELLOW}Press Ctrl+C to shutdown${NC}"
echo ""

# Start DroneBrain with passed arguments
python3 main.py "$@"
