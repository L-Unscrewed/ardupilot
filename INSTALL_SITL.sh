#!/bin/bash
# Virtual Anchor SITL Installation Script
# Run this with: bash INSTALL_SITL.sh

set -e  # Exit on error

echo "==========================================="
echo "Virtual Anchor SITL Setup"
echo "==========================================="

# Step 1: Install system dependencies
echo ""
echo "Step 1: Installing system dependencies..."
echo "This will ask for your sudo password"
sudo apt-get update
sudo apt-get install -y \
    g++ \
    python3-dev \
    python3-venv \
    python3-full \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    git \
    cmake \
    gawk \
    ccache

echo "✓ System dependencies installed"

# Step 2: Create Python virtual environment
echo ""
echo "Step 2: Creating Python virtual environment..."
cd ~/ardupilot
python3 -m venv sitl-env
source sitl-env/bin/activate

echo "✓ Virtual environment created"

# Step 3: Install Python packages
echo ""
echo "Step 3: Installing Python packages for SITL..."
pip install --upgrade pip
pip install MAVProxy pymavlink pexpect empy==3.3.4 dronecan pygame lxml numpy

echo "✓ Python packages installed"

# Step 4: Build ArduRover
echo ""
echo "Step 4: Building ArduRover for SITL..."
./waf configure --board sitl
./waf rover

echo ""
echo "==========================================="
echo "✓ Installation Complete!"
echo "==========================================="
echo ""
echo "To use SITL, you need to activate the virtual environment:"
echo "  cd ~/ardupilot"
echo "  source sitl-env/bin/activate"
echo ""
echo "Then run SITL:"
echo "  cd Tools/autotest"
echo "  ./sim_vehicle.py -v Rover --console --map"
echo ""
echo "In another terminal, run the test:"
echo "  cd ~/ardupilot"
echo "  source sitl-env/bin/activate"
echo "  python3 test_vanc.py"
echo ""
echo "==========================================="
