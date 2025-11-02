# Virtual Anchor SITL Testing - Quick Start Guide

## Prerequisites Installation

### Option 1: Automated Installation (Recommended)

Run the installation script:

```bash
cd ~/ardupilot
bash INSTALL_SITL.sh
```

This will:
- Install system dependencies (requires sudo password)
- Create a Python virtual environment
- Install MAVProxy and pymavlink
- Build ArduRover for SITL

**Takes 5-10 minutes on first run.**

### Option 2: Manual Installation

```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install -y g++ python3-dev python3-venv python3-full \
    python3-opencv python3-wxgtk4.0 python3-matplotlib python3-lxml \
    python3-pygame git cmake gawk ccache

# Create virtual environment
cd ~/ardupilot
python3 -m venv sitl-env
source sitl-env/bin/activate

# Install Python packages
pip install --upgrade pip
pip install MAVProxy pymavlink

# Build ArduRover
./waf configure --board sitl
./waf rover
```

Expected output: `Build finished successfully`

## Important: Always Activate Virtual Environment

Before running SITL or test scripts:

```bash
cd ~/ardupilot
source sitl-env/bin/activate
```

You'll see `(sitl-env)` appear in your terminal prompt.

## Test Method 1: Automated Test Script (Recommended)

### Terminal 1: Launch SITL
```bash
cd ~/ardupilot
source sitl-env/bin/activate  # Activate virtual environment
cd Tools/autotest
./sim_vehicle.py -v Rover --console --map
```

Wait for:
- "GPS lock" message
- EKF status shows GOOD
- Home position set

### Terminal 2: Run Test Script
```bash
cd ~/ardupilot
source sitl-env/bin/activate  # Activate virtual environment
python3 test_vanc.py
```

The script will:
1. Connect to SITL
2. Switch to VANC mode
3. Set anchor 30m North
4. Monitor for 60 seconds showing real-time status

### Expected Output:
```
[10s] Dist: 19.85m | Error: -0.15m | Speed: 0.05m/s | ✓ WITHIN TOLERANCE
[11s] Dist: 20.12m | Error: +0.12m | Speed: 0.03m/s | ✓ WITHIN TOLERANCE
```

## Test Method 2: Manual Testing

### Step 1: Launch SITL
```bash
cd ~/ardupilot
source sitl-env/bin/activate  # Activate virtual environment
cd Tools/autotest
./sim_vehicle.py -v Rover --console --map
```

### Step 2: Configure Parameters
In the MAVProxy console:
```bash
# Basic parameters
param set VANC_ROPE_LEN 20.0
param set VANC_TOLERANCE 2.0
param set VANC_SPEED 1.5

# Azimuth thruster parameters
param set VANC_MIN_THR 0.3

# PID tuning
param set VANC_PID_P 0.5
param set VANC_PID_I 0.02
param set VANC_PID_D 0.2
param set VANC_PID_IMAX 0.5

param fetch
```

### Step 3: Switch to VANC Mode
```bash
mode VANC
```

Expected: `Virtual Anchor mode entered`

### Step 4: Set Anchor Point
```bash
# Get current position
wp list

# Set anchor using guided mode approach:
# First switch to GUIDED
mode GUIDED

# Move to a point (this will be our test)
guided LATLON -35.3632607 149.1652351

# Wait for arrival, then switch back to VANC
mode VANC

# The vehicle should now try to maintain distance from that point
```

### Step 5: Monitor Telemetry
Watch the MAVProxy console for messages like:
```
VANC: dist=19.5m err=-0.5m spd=0.15 I=0.02
```

**Interpretation:**
- `dist`: Current distance to anchor
- `err`: Error from target (20m rope length)
- `spd`: Desired speed (PID output)
- `I`: Integrator value

## What to Look For

### Good Behavior ✓
- Vehicle navigates toward anchor point
- Settles within 18-22m (rope_len ± tolerance)
- Speed drops to near zero when in tolerance
- Minimal oscillation
- Smooth transitions

### Problem Signs ✗
- Continuous oscillation (back and forth)
- Overshoots significantly (>4m past anchor)
- Doesn't reach tolerance band
- Integrator keeps growing (>0.5)
- Erratic movements

## PID Tuning in SITL

If you see problems, adjust PID gains:

### Oscillation Problem
```bash
param set VANC_PID_P 0.3    # Reduce P gain
param set VANC_PID_D 0.3    # Increase D gain
```

### Slow Response
```bash
param set VANC_PID_P 0.7    # Increase P gain
```

### Steady-State Error (doesn't reach 20m)
```bash
param set VANC_PID_I 0.05   # Increase I gain
```

### Integrator Windup (I value grows >0.5)
```bash
param set VANC_PID_IMAX 0.3  # Reduce max integrator
param set VANC_PID_I 0.01    # Reduce I gain
```

## Advanced: Testing with Current/Wind

SITL can simulate wind. Add to sim_vehicle.py command:
```bash
./sim_vehicle.py -v Rover --console --map \
    --defaults Tools/autotest/default_params/rover.parm \
    -A "--wind=5,270"  # 5 m/s wind from West
```

This tests the I-term's ability to compensate for constant drift.

## Troubleshooting

### "Permission denied" on waf
```bash
chmod +x waf
```

### "Could not configure C++ compiler"
```bash
sudo apt-get install g++
```

### MAVProxy not found
```bash
export PATH=$PATH:~/.local/bin
# Or restart terminal after pip install
```

### SITL doesn't start
```bash
# Clean build and retry
./waf clean
./waf configure --board sitl
./waf rover
```

### Can't connect to SITL
- Check SITL is running (should see heartbeat messages)
- Default connection: udp:127.0.0.1:14550
- Try: `mavproxy.py --master=udp:127.0.0.1:14550`

## Data Logging

SITL logs are saved to: `Tools/autotest/logs/`

View logs:
```bash
cd Tools/autotest/logs
# Find latest log (DATE-TIME format)
ls -lt | head

# Replay log in MAVProxy
mavlogdump.py --type STATUSTEXT LOGFILE.bin
```

## Next Steps After SITL Testing

1. Verify PID tuning works well
2. Test with simulated wind/current
3. Note optimal parameter values
4. Flash to real hardware with same parameters
5. Test in calm water first
6. Gradually increase sea state

## Quick Reference: Parameter Summary

| Parameter | Azimuth Thrusters | Traditional Steering |
|-----------|-------------------|---------------------|
| VANC_MIN_THR | 0.3 | 0.0 |
| VANC_PID_P | 0.5 | 0.5 |
| VANC_PID_I | 0.02 | 0.0 |
| VANC_PID_D | 0.2 | 0.1 |
| VANC_PID_IMAX | 0.5 | 0.5 |
| VANC_ROPE_LEN | 20.0 | 20.0 |
| VANC_TOLERANCE | 2.0 | 2.0 |
| VANC_SPEED | 1.5 | 1.5 |
