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

def send_vision_position_estimate(master, x, y, z, roll, pitch, yaw):
    """
    Send vision position estimate data to the vehicle.
    """
    current_time_us = int(time.time() * 1e6)
    tracker_confidence = 3  # Simulate a tracking confidence level (1-3)
    cov_scale = pow(10, 3 - tracker_confidence)
    covariance = [0.01 * cov_scale] * 21  # Simplified diagonal covariance
    
    master.mav.vision_position_estimate_send(
        current_time_us,  # Timestamp (microseconds since UNIX epoch)
        x,                # Global X position
        y,                # Global Y position
        z,                # Global Z position
        roll,             # Roll angle in radians
        pitch,            # Pitch angle in radians
        yaw,              # Yaw angle in radians
        covariance        # Covariance matrix upper right triangular (first six rows of 6x6 matrix)
    )
    print(f"Sent vision position estimate: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")

def set_ekf_home(master, latitude, longitude, altitude):
    """
    Set the EKF home position.
    """
    master.mav.set_gps_global_origin_send(
        master.target_system,
        int(latitude * 1e7),
        int(longitude * 1e7),
        int(altitude * 1000)
    )
    master.mav.set_home_position_send(
        master.target_system,
        int(latitude * 1e7),
        int(longitude * 1e7),
        int(altitude * 1000),
        0, 0, 0,  # x, y, z positions (local frame)
        1, 0, 0,  # q (w, x, y, z quaternion components, here unit quaternion)
        0, 0, 0,  # approach x, y, z
    )
    print(f"Set EKF home position: lat={latitude}, lon={longitude}, alt={altitude}")

def main():
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    # Wait for a heartbeat before sending commands
    master.wait_heartbeat()
    print("Heartbeat received from vehicle")

    # Set required parameters
    params = {
        'AHRS_EKF_TYPE': 2,
        'EK2_ENABLE': 1,
        'EK3_ENABLE': 0,
        'GPS_TYPE': 0,
        'EK2_GPS_TYPE': 3,
        'EK2_POSNE_M_NSE': 0.1,
        'EK2_VELD_M_NSE': 0.1,
        'EK2_VELNE_M_NSE': 0.1,
        'COMPASS_ENABLE': 0,
        'COMPASS_USE': 0,
        'COMPASS_USE2': 0,
        'COMPASS_USE3': 0
    }

    for param_id, param_value in params.items():
        set_param(master, param_id, param_value)

    # Fake GPS coordinates for EKF home (latitude, longitude, altitude)
    home_latitude = 47.397742  # degrees
    home_longitude = 8.545594  # degrees
    home_altitude = 488.0  # meters

    # Set EKF home position
    set_ekf_home(master, home_latitude, home_longitude, home_altitude)

    # Fake vision position data (x, y, z, roll, pitch, yaw)
    fake_x = 0.0  # meters
    fake_y = 0.0  # meters
    fake_z = 0.0  # meters
    fake_roll = 0.0  # radians
    fake_pitch = 0.0  # radians
    fake_yaw = 0.0  # radians

    try:
        while True:
            # Send vision position estimate data every second
            send_vision_position_estimate(master, fake_x, fake_y, fake_z, fake_roll, fake_pitch, fake_yaw)
            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting...")
        master.close()

if __name__ == '__main__':
    main()
