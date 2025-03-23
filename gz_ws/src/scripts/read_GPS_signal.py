from pymavlink import mavutil

class BoatController:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.conn = None

    def connect(self):
        # Create the connection
        self.conn = mavutil.mavlink_connection(self.connection_string)
        # Wait for a heartbeat before sending commands
        self.conn.wait_heartbeat()
        print("Connected to vehicle.")

    def disconnect(self):
        if self.conn:
            self.conn.close()
            self.conn = None
            print("Disconnected from vehicle.")

    def read_gps_data(self):
        self.connect()
        try:
            while True:
                # Wait for a GPS_RAW_INT message
                msg = self.conn.recv_match(type='GPS_RAW_INT', blocking=True)
                if msg:
                    # Extract GPS data
                    latitude = msg.lat / 1e7
                    longitude = msg.lon / 1e7
                    altitude = msg.alt / 1e3  # in meters
                    satellites_visible = msg.satellites_visible
                    fix_type = msg.fix_type

                    # Determine if the GPS signal is reliable
                    gps_reliable = fix_type >= 3  # A 3D fix or higher is considered reliable

                    # Print the GPS data
                    print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}m")
                    print(f"Satellites Visible: {satellites_visible}")
                    print(f"GPS Signal Reliable: {'Yes' if gps_reliable else 'No'}")

        except KeyboardInterrupt:
            # Handle user interruption (Ctrl+C)
            print("Terminating GPS data reading.")
        finally:
            self.disconnect()

if __name__ == "__main__":
    boat_controller = BoatController('udpin:0.0.0.0:14550')
    boat_controller.read_gps_data()
