import sys
import argparse
from pymavlink import mavutil
import time
import math
import threading

# Connect to the vehicle
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

def arm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    print("Attempting to arm the vehicle...")
    timeout = time.time() + 10  # 10 second timeout for arming
    armed = False

    while not armed and time.time() < timeout:
        message = master.recv_match(type='HEARTBEAT', blocking=True)
        if message:
            print(message.to_dict())
            armed = message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                print('Vehicle is armed!')
                break

    if not armed:
        print('Failed to arm the vehicle. Check pre-arm conditions and messages.')

def set_guided_mode():
    mode = 'GUIDED'
    if mode not in master.mode_mapping():
        print(f"Mode {mode} not found in mode mapping. Exiting...")
        return
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Mode set to GUIDED.")

def get_initial_position():
    print("Waiting for GPS position...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            initial_lat = msg.lat / 1e7
            initial_lon = msg.lon / 1e7
            print(f"Initial position: lat={initial_lat}, lon={initial_lon}")
            return initial_lat, initial_lon

def send_target_position(lat, lon):
    # Send the target position to the vehicle
    master.mav.set_position_target_global_int_send(
        0,                         # time_boot_ms
        master.target_system,      # target_system
        master.target_component,   # target_component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000,        # type_mask (only positions enabled)
        int(lat * 1e7),            # lat_int - X Position in WGS84 frame in 1e7 * degrees
        int(lon * 1e7),            # lon_int - Y Position in WGS84 frame in 1e7 * degrees
        0,                         # alt
        0, 0, 0,                   # X, Y, Z velocity in m/s (not used)
        0, 0, 0,                   # afx, afy, afz acceleration (not used)
        0, 0)                      # yaw, yaw rate (not used)

def follow_target_positions(initial_lat, initial_lon, positions):
    for idx, (x, y) in enumerate(positions):
        lat = initial_lat + (y / 111320)  # Latitude degree per meter
        lon = initial_lon + (x / (40075000 * (1 / 360) * math.cos(math.radians(initial_lat))))  # Longitude degree per meter
        
        send_target_position(lat, lon)
        print(f"Sent target position {idx + 1}/{len(positions)}: lat={lat}, lon={lon}")

        # Wait for the rover to reach the position with a timeout
        start_time = time.time()
        timeout = 60  # 60 seconds timeout
        while time.time() - start_time < timeout:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7

                # Check if the BlueBoat has reached the target position
                if abs(current_lat - lat) < 0.00005 and abs(current_lon - lon) < 0.00005:  # Increased tolerance
                    print(f"Reached target position {idx + 1}/{len(positions)}: lat={lat}, lon={lon}")
                    break
        else:
            print(f"Timeout reached while waiting for target position {idx + 1}/{len(positions)}")

        time.sleep(2)

def print_odometry():
    while True:
        msg = master.recv_match(blocking=True)
        if not msg:
            continue

        if msg.get_type() == "LOCAL_POSITION_NED":
            x = msg.x
            y = msg.y
            z = msg.z
            vx = msg.vx
            vy = msg.vy
            vz = msg.vz
            print(f"Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            print(f"Velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")
        
        elif msg.get_type() == "ATTITUDE":
            roll = msg.roll
            pitch = msg.pitch
            yaw = msg.yaw
            rollspeed = msg.rollspeed
            pitchspeed = msg.pitchspeed
            yawspeed = msg.yawspeed
            print(f"Orientation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
            print(f"Angular velocity: rollspeed={rollspeed:.2f}, pitchspeed={pitchspeed:.2f}, yawspeed={yawspeed:.2f}")

        time.sleep(1)  # Print odometry at 1 Hz

def main_loop():
    initial_lat, initial_lon = get_initial_position()
    print("Entering main loop. Waiting for new target positions...")

    while True:
        user_input = input("Enter target positions as 'x1,y1;x2,y2;...': ")
        positions = []
        
        if user_input.strip():
            try:
                pairs = user_input.split(';')
                for pair in pairs:
                    x, y = map(float, pair.split(','))
                    positions.append((x, y))
                follow_target_positions(initial_lat, initial_lon, positions)
            except ValueError:
                print("Invalid input format. Please enter positions as 'x1,y1;x2,y2;...'.")
        else:
            print("No input provided. Waiting for new target positions...")

if __name__ == "__main__":
    arm_vehicle()
    set_guided_mode()

    odometry_thread = threading.Thread(target=print_odometry)
    odometry_thread.daemon = True
    odometry_thread.start()

    main_loop()
