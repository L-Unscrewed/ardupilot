#!/usr/bin/env python3
"""
Virtual Anchor Mode Test Script for SITL
Run this after launching sim_vehicle.py
"""

from pymavlink import mavutil
import time
import math

# Connect to SITL
print("Connecting to SITL...")
master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
master.wait_heartbeat()
print("Connected! System ID:", master.target_system)

# Wait for GPS lock
print("\nWaiting for GPS lock...")
while True:
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
    if msg and msg.fix_type >= 3:
        print(f"GPS locked! Satellites: {msg.satellites_visible}, HDOP: {msg.eph/100.0}")
        break

# Get home position
print("\nGetting current position...")
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    4,
    1
)

time.sleep(1)

msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
if msg:
    current_lat = msg.lat / 1e7
    current_lon = msg.lon / 1e7
    current_alt = msg.alt / 1000.0
    print(f"Current position: {current_lat:.7f}, {current_lon:.7f}, {current_alt:.1f}m")
else:
    print("ERROR: Could not get current position")
    exit(1)

# Calculate anchor point 30m North
# 1 degree latitude ≈ 111,111 meters
anchor_lat = current_lat + (30.0 / 111111.0)
anchor_lon = current_lon
anchor_alt = current_alt

print(f"\nAnchor point (30m North): {anchor_lat:.7f}, {anchor_lon:.7f}")

# Switch to VANC mode (mode 17)
print("\nSwitching to Virtual Anchor mode...")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    17  # VANC mode number
)

time.sleep(1)

# Send MAV_CMD_DO_SET_VIRTUAL_ANCHOR command
print("Setting virtual anchor point...")
master.mav.command_int_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
    243,  # MAV_CMD_DO_SET_VIRTUAL_ANCHOR
    0,    # current (0=not current waypoint)
    0,    # autocontinue
    0,    # param1 (unused)
    0,    # param2 (unused)
    0,    # param3 (unused)
    0,    # param4 (unused)
    int(anchor_lat * 1e7),  # x: latitude in 1e7 degrees
    int(anchor_lon * 1e7),  # y: longitude in 1e7 degrees
    anchor_alt              # z: altitude
)

# Wait for command ACK
print("Waiting for anchor set confirmation...")
msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if msg and msg.command == 243:
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✓ Virtual anchor set successfully!")
    else:
        print(f"✗ Command failed with result: {msg.result}")
else:
    print("✗ No acknowledgment received")

# Monitor position for 60 seconds
print("\n" + "="*70)
print("MONITORING VIRTUAL ANCHOR MODE (60 seconds)")
print("="*70)
print(f"Target: Stay within {20.0}m ± {2.0}m of anchor point")
print(f"Min thrust: 0.3 m/s (azimuth thruster mode)")
print("="*70)

start_time = time.time()
last_print = 0

while time.time() - start_time < 60:
    # Request position updates
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)

    if msg and time.time() - last_print >= 1.0:
        # Calculate distance to anchor
        vehicle_lat = msg.lat / 1e7
        vehicle_lon = msg.lon / 1e7

        # Simple distance calculation (accurate for small distances)
        dlat = (anchor_lat - vehicle_lat) * 111111.0
        dlon = (anchor_lon - vehicle_lon) * 111111.0 * math.cos(math.radians(vehicle_lat))
        distance = math.sqrt(dlat**2 + dlon**2)

        # Get velocity
        vx = msg.vx / 100.0  # cm/s to m/s
        vy = msg.vy / 100.0
        speed = math.sqrt(vx**2 + vy**2)

        # Display status
        elapsed = int(time.time() - start_time)
        error = distance - 20.0

        # Color coding for terminal
        if abs(error) <= 2.0:
            status = "✓ WITHIN TOLERANCE"
        elif error > 2.0:
            status = "⚠ TOO FAR"
        else:
            status = "⚠ TOO CLOSE"

        print(f"[{elapsed:2d}s] Dist: {distance:5.2f}m | Error: {error:+5.2f}m | Speed: {speed:4.2f}m/s | {status}")

        last_print = time.time()

print("\n" + "="*70)
print("Test complete! Check the output above to verify:")
print("  1. Vehicle navigated toward anchor point")
print("  2. Settled within tolerance band (18-22m)")
print("  3. Maintained position with minimal speed when in tolerance")
print("  4. PID control kept distance error small")
print("="*70)

master.close()
