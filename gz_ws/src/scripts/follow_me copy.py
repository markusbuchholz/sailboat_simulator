from pymavlink import mavutil
import time
import math

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()

def arm_vehicle():
    # Try to arm the vehicle
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # Wait and check for arming confirmation
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

def set_follow_mode():
    # Set the mode to FOLLOW
    mode = 'FOLLOW'
    if mode not in master.mode_mapping():
        print(f"Mode {mode} not found in mode mapping. Exiting...")
        return
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Mode set to FOLLOW.")

def send_follow_target(lat, lon, alt=0):
    # Send the follow target position to the vehicle
    master.mav.follow_target_send(
        int(time.time() * 1000),  # timestamp (milliseconds since system boot)
        0,                        # est_capabilities (not used)
        int(lat * 1e7),           # latitude (WGS84), in 1e7 degrees
        int(lon * 1e7),           # longitude (WGS84), in 1e7 degrees
        int(alt * 1000),          # altitude (AMSL, WGS84), in millimeters
        [0, 0, 0],                # velocity in X, Y, Z direction (m/s)
        [0, 0, 0],                # acceleration in X, Y, Z direction (m/s^2)
        [1, 0, 0, 0],             # attitude quaternion (w, x, y, z)
        [0, 0, 0],                # body rates (roll, pitch, yaw)
        [0, 0, 0],                # position covariance
        0                         # custom state (not used)
    )

def follow_me():
    # Example target positions to follow (in degrees)
    targets = [
        (55.9954164, -3.301032),  # Initial position
        (55.9954164 + 0.0001, -3.301032 + 0.0001),
        (55.9954164 + 0.0002, -3.301032 + 0.0002),
        (55.9954164 + 0.0003, -3.301032 + 0.0003)
    ]
    
    for lat, lon in targets:
        send_follow_target(lat, lon)
        print(f"Sent follow target: lat={lat}, lon={lon}")
        time.sleep(1)  # Send at 1 Hz

if __name__ == "__main__":
    arm_vehicle()
    set_follow_mode()
    follow_me()
