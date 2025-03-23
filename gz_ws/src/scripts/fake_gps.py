import time
from pymavlink import mavutil

def set_param(master, param_id, param_value):
    """
    Set a parameter value on the vehicle.
    """
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    print(f"Set {param_id} to {param_value}")

def send_gps_data(master, latitude, longitude, altitude):
    """
    Send fake GPS data to the vehicle.
    """
    gps_msg = master.mav.gps_raw_int_encode(
        int(time.time() * 1000),  # time_boot_ms (timestamp in milliseconds)
        3,  # fix_type (3 = 3D fix)
        int(latitude * 1e7),  # lat (latitude in 1E7 degrees)
        int(longitude * 1e7),  # lon (longitude in 1E7 degrees)
        int(altitude * 1000),  # alt (altitude in millimeters)
        65535,  # eph (GPS HDOP horizontal dilution of position in cm (uint16))
        65535,  # epv (GPS VDOP vertical dilution of position in cm (uint16))
        0,  # vel (GPS ground speed in cm/s (uint16))
        0,  # cog (Course over ground in degrees * 100 (uint16))
        255,  # satellites_visible (Number of satellites visible (uint8))
        0,  # alt_ellipsoid (altitude above the ellipsoid in millimeters (int32))
        0,  # h_acc (horizontal accuracy in millimeters (uint32))
        0,  # v_acc (vertical accuracy in millimeters (uint32))
        0,  # vel_acc (velocity accuracy in centimeters (uint32))
        0   # hdg_acc (heading accuracy in degrees * 1E5 (uint32))
    )
    master.mav.send(gps_msg)
    print(f"Sent GPS data: lat={latitude}, lon={longitude}, alt={altitude}")

def main():
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    # Wait for a heartbeat before sending commands
    master.wait_heartbeat()
    print("Heartbeat received from vehicle")

    # Set parameters to use MAVLink GPS
    set_param(master, 'GPS_TYPE', 14)  # GPS_TYPE set to MAVLink
    set_param(master, 'GPS_TYPE2', 14) # GPS_TYPE2 set to MAVLink (if dual GPS is used)

    # Fake GPS coordinates (latitude, longitude, altitude)
    fake_latitude = 47.397742  # degrees
    fake_longitude = 8.545594  # degrees
    fake_altitude = 488.0  # meters

    try:
        while True:
            # Send fake GPS data every second
            send_gps_data(master, fake_latitude, fake_longitude, fake_altitude)
            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting...")
        master.close()

if __name__ == '__main__':
    main()
