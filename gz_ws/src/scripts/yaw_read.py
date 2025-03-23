from pymavlink import mavutil
import math

# Connect to the vehicle
connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the first heartbeat to know the target system ID
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

def get_yaw_deg():
    """
    Fetches the yaw (heading) of the vehicle from the MAVLink ATTITUDE message.
    """
    # Request data from the vehicle
    msg = connection.recv_match(type='ATTITUDE', blocking=True)
    
    if msg is not None:
        # Yaw in radians
        yaw_rad = msg.yaw

        # Convert yaw to degrees
        yaw_deg = math.degrees(yaw_rad)

        # Normalize yaw to 0-360 degrees
        yaw_deg_normalized = yaw_deg % 360

        print("Current yaw: {:.2f} degrees".format(yaw_deg_normalized))
    else:
        print("No attitude data received")

# Continuously print the yaw
while True:
    get_yaw_deg()
