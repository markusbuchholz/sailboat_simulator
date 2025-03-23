import time
from pymavlink import mavutil

# Establish communication
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
            armed = message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                print('Vehicle is armed!')
            else:
                print(f"Arming failed. Current base mode: {message.base_mode}")

    if not armed:
        print('Failed to arm the vehicle. Check pre-arm conditions and messages.')

def set_mode(mode_name):
    if mode_name not in master.mode_mapping():
        print(f"Mode {mode_name} not found in mode mapping. Exiting...")
        return
    mode_id = master.mode_mapping()[mode_name]
    master.set_mode(mode_id)
    print(f"Mode set to {mode_name}.")

def get_current_position():
    print("Waiting for GPS position...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            print(f"Current position: lat={current_lat}, lon={current_lon}")
            return current_lat, current_lon

def hold_position(lat, lon):
    # Send the target position to the vehicle
    master.mav.set_position_target_global_int_send(
        0,                         # time_boot_ms
        master.target_system,      # target_system
        master.target_component,   # target_component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000,        # type_mask (only positions enabled)
        int(lat * 1e7),            # lat_int - X Position in WGS84 frame in 1e7 * degrees
        int(lon * 1e7),            # lon_int - Y Position in WGS84 frame in 1e7 * degrees
        0,                         # alt (set to 0 for ground vehicles)
        0, 0, 0,                   # X, Y, Z velocity in m/s (not used)
        0, 0, 0,                   # afx, afy, afz acceleration (not used)
        0, 0)                      # yaw, yaw rate (not used)

def main_loop():
    current_lat, current_lon = get_current_position()
    set_mode('LOITER')
    hold_position(current_lat, current_lon)
    
    # while True:
    #     user_input = input("Enter 'hold' to hold current position, 'guided' to switch to GUIDED mode, or new position as 'lat,lon': ")
        
    #     if user_input.strip().lower() == 'hold':
    #         print("Holding current position.")
    #         hold_position(current_lat, current_lon)
    #     elif user_input.strip().lower() == 'guided':
    #         print("Switching to GUIDED mode.")
    #         set_mode('GUIDED')
    #     else:
    #         try:
    #             lat, lon = map(float, user_input.split(','))
    #             current_lat, current_lon = lat, lon
    #             hold_position(current_lat, current_lon)
    #             print(f"Holding new position: lat={current_lat}, lon={current_lon}")
    #         except ValueError:
    #             print("Invalid input format. Please enter 'hold', 'guided', or a new position as 'lat,lon'.")

if __name__ == "__main__":
    arm_vehicle()
    main_loop()
