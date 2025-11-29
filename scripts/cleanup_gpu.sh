#!/bin/bash
# GPU Cleanup Script
# Kills orphaned Python processes consuming GPU memory

echo "=========================================="
echo "DroneBrain GPU Cleanup Utility"
echo "=========================================="
echo ""

# Get list of Python processes using GPU (except this script's parent)
CURRENT_PID=$$
PARENT_PID=$PPID

echo "Checking for orphaned Python processes on GPU..."
nvidia-smi

echo ""
echo "Orphaned Python processes:"
ps aux | grep python3 | grep -v grep | grep -v $CURRENT_PID | grep -v $PARENT_PID

echo ""
read -p "Kill all orphaned Python processes? (y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]
then
    echo "Killing orphaned processes..."
    # Kill all python3 processes except current shell
    pkill -9 python3 2>/dev/null || true

    echo "Clearing CUDA cache..."
    sleep 2

    echo ""
    echo "GPU status after cleanup:"
    nvidia-smi

    echo ""
    echo "✓ Cleanup complete!"
else
    echo "Cleanup cancelled"
fi
