import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from time import sleep, time
import math
from threading import Thread
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float64MultiArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MavlinkControlNode(Node):
    def __init__(self):
        super().__init__('mavlink_control_node')
        
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.callback_group = ReentrantCallbackGroup()
        self.vehicle = self.create_connection()
        self.get_logger().info("Waiting for heartbeat...")
        self.vehicle.wait_heartbeat()
        self.get_logger().info("Heartbeat received")

        self.set_vehicle_parameter('WPNAV_SPEED', 15.0)
        self.set_vehicle_parameter('WPNAV_SPEED_DN', 25.0)
        self.set_vehicle_parameter('WPNAV_SPEED_UP', 25.0)

        self.control_thread = Thread(target=self.main_control_loop)
        self.control_thread.start()
        
        self.waypoint_publisher = self.create_publisher(Float64MultiArray, '/waypoints', qos_profile=self.custom_qos, callback_group=self.callback_group)
        
        self.last_published_position = None
        self.dynamic_last_published_position = None
        self.check_pos = None
        self.rov_error = 0.0
        
        
    def get_vehicle_status(self):

        # Wait for the heartbeat message to find the system ID
        self.vehicle.wait_heartbeat()

        print("Connected to vehicle with system ID:", self.vehicle.target_system)

        # Request a data stream to get the vehicle's status
        # MAV_DATA_STREAM_ALL = 0
        # rate = 1 (1 Hz)
        self.vehicle.mav.request_data_stream_send(self.vehicle.target_system, self.vehicle.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

        # Wait for the first status message
        while True:
            msg = self.vehicle.recv_match(type=['HEARTBEAT', 'SYS_STATUS'], blocking=True)
            if msg:
                print(f"Received message: {msg}")
                break


    def publish_waypoint(self, position):
        self.get_logger().info(f"Publishing new waypoint ############################: {position}")
        #self.get_logger().info(f"Current state -----> : {self.check_pos}")
        msg = Float64MultiArray()
        msg.data = [position['y'], position['x'], position['z'], position['yaw']]
        self.waypoint_publisher.publish(msg)
        self.last_published_position = position
        self.get_logger().info(f"Waypoint published at: {position}")
        
        
    def set_guided_mode_hold_position_local(self):
        self.get_logger().info("Setting GUIDED mode and holding position...")
        current_state = self.wait_for_valid_position()

        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED mode
        )

        self.vehicle.mav.set_position_target_local_ned_send(
            0,
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            current_state['x'],
            current_state['y'],
            current_state['z'],
            0, 0, 0,
            0, 0, 0,
            current_state['yaw'],
            0
        )
        return current_state
    
    # 0.5 OK for Boat with PID
    def has_moved_significantly(self, current_position, threshold=0.01): 
        
        # if self.last_published_position is None:
        #     return True
        if self.dynamic_last_published_position is None:
            return True
        #3D
        # distance = math.sqrt(
        #     (current_position['x'] - abs(self.dynamic_last_published_position['x']))**2 +
        #     (current_position['y'] - abs(self.dynamic_last_published_position['y']))**2 +
        #     (current_position['z'] - abs(self.dynamic_last_published_position['z']))**2
        # )
        distance = math.sqrt(
            (current_position['x'] - abs(self.dynamic_last_published_position['x']))**2 +
            (current_position['y'] - abs(self.dynamic_last_published_position['y']))**2 
        )
        
        #self.get_logger().info(f"DISTANCE ******************: distance={distance}")
        if distance <= threshold:
            self.dynamic_last_published_position = current_position
            
            
        return distance >= threshold

    def create_connection(self):
        return mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    def set_vehicle_mode(self, mode):
        mode_id = self.vehicle.mode_mapping()[mode]
        self.vehicle.set_mode(mode_id)
        self.get_logger().info(f"Vehicle mode set to {mode}")

    def set_vehicle_parameter(self, param_name, param_value):
        self.vehicle.mav.param_set_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            param_name.encode('utf-8'),
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        self.get_logger().info(f"Set parameter {param_name} to {param_value}")

    def arm_vehicle(self):
        self.vehicle.arducopter_arm()
        self.vehicle.motors_armed_wait()
        self.get_logger().info("Vehicle armed")

    def disarm_vehicle(self):
        self.vehicle.arducopter_disarm()
        self.vehicle.motors_disarmed_wait()
        self.get_logger().info("Vehicle disarmed")
        
    def wait_for_valid_position(self, max_attempts=30):
        for attempt in range(max_attempts):
            current_state = self.fetch_current_state_local()
            if current_state['x'] != 0 or current_state['y'] != 0 or current_state['z'] != 0:
                return current_state
            self.get_logger().warn(f"Invalid position data, retrying... ({attempt + 1}/{max_attempts})")
            sleep(1)
        raise TimeoutError("Failed to get a valid position")

    def has_reached_position(self, current, target, threshold=1.0):
        #self.get_logger().debug(f"Current Position: x={current['x']}, y={current['y']}, z={current['z']}")
        #self.get_logger().debug(f"Target Position: x={target['x']}, y={target['y']}, z={target['z']}")

        distance = math.sqrt((current['x'] - target['x'])**2 + (current['y'] - target['y'])**2 + (current['z'] - target['z'])**2)
        #self.get_logger().info(f"Distance to target: {distance} (Threshold: {threshold})")
        # if distance < 2.0:
        #     self.publish_waypoint(current_position)
        return distance < threshold

    def fetch_current_state_local(self):
        default_position = {'x': 0, 'y': 0, 'z': 0}
        default_orientation = {'yaw': 0}

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            0, 0, 0, 0, 0, 0, 0
        )
        position_message = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=7)
        if position_message:
            position = {'x': position_message.x, 'y': position_message.y, 'z': position_message.z}
        else:
            position = default_position

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            0, 0, 0, 0, 0, 0, 0
        )
        attitude_message = self.vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=7)
        if attitude_message:
            orientation = {'yaw': attitude_message.yaw}
        else:
            orientation = default_orientation

        return {
            'x': position['x'],
            'y': position['y'],
            'z': -position['z'],
            'yaw': orientation['yaw']
        }
    #yaw30 =0.5236)
    #yaw60=1.0472
    #yaw90=1.5708
    #yaw180=3.1415    
    def send_position_request(self, x, y, z, yaw):
        # Correct type mask for controlling position (X, Y, Z) and yaw
        type_mask = 0b100111111000  # Control positions (x, y, z) and yaw, ignore velocity, acceleration, and yaw rate
        coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        # If no yaw is specified, default to 0.0 (or you could fetch the current yaw if needed)
        if yaw is None:
            yaw = 0.0

        self.vehicle.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.vehicle.target_system,  # Target system
            self.vehicle.target_component,  # Target component
            coordinate_frame,  # Coordinate frame
            type_mask,  # Type mask (specifies which fields are active)
            x, y, z,  # Positions
            0, 0, 0,  # Velocities (ignored)
            0, 0, 0,  # Accelerations (ignored)
            yaw, 0  # Yaw (controlled) and yaw rate (ignored)
        )
        self.get_logger().info(f"Position request sent: x={x}, y={y}, z={z}, yaw={yaw}")
        
    def main_control_loop(self):
        try:
            #self.get_vehicle_status()
            current_state = self.set_guided_mode_hold_position_local()
           # self.check_pos = current_state
            self.arm_vehicle()
            self.last_published_position = current_state

            #rough sea {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 4.5, 'z': current_state['z'] - 1.5, 'yaw':1.6},
            #rough sea first_waypoint = True
            waypoints = [
                #1
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 3.5, 'z': current_state['z'] - 1.5, 'yaw':1.6},
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 0.0, 'z': current_state['z'] - 1.5, 'yaw':5.6},
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 3.5, 'z': current_state['z'] - 1.5, 'yaw':1.6},
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 0.0, 'z': current_state['z'] - 1.5, 'yaw':5.6},
                {'x': current_state['x'] + 2.5, 'y': current_state['y'] + 0.5, 'z': current_state['z'] - 1.5, 'yaw':-0.2},
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 0.5, 'z': current_state['z'] - 1.5, 'yaw':3.6},
                {'x': current_state['x'] - 1.5, 'y': current_state['y'] + 0.0, 'z': current_state['z'] - 1.5, 'yaw':3.6},
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 0.0, 'z': current_state['z'] - 1.5, 'yaw':-0.2},
                
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 3.0, 'z': current_state['z'] - 1.5, 'yaw':1.6},
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 6.5, 'z': current_state['z'] - 1.5, 'yaw':1.6},
                {'x': current_state['x'] + 4.0, 'y': current_state['y'] + 6.5, 'z': current_state['z'] - 1.5, 'yaw':-0.2},
                {'x': current_state['x'] + 4.0, 'y': current_state['y'] + 2.0, 'z': current_state['z'] - 1.5, 'yaw':5.6},
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 2.0, 'z': current_state['z'] - 1.5, 'yaw':3.0},
                
                
                
                #2
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 3.2, 'z': current_state['z'] - 1.5, 'yaw':1.6},
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 3.3, 'z': current_state['z'] - 1.5, 'yaw':1.6},              
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 6.5, 'z': current_state['z'] - 1.5, 'yaw':1.6},
                {'x': current_state['x'] + 4.0, 'y': current_state['y'] + 6.5, 'z': current_state['z'] - 1.5, 'yaw':-0.2},
                {'x': current_state['x'] + 4.0, 'y': current_state['y'] + 2.0, 'z': current_state['z'] - 1.5, 'yaw':5.6},
                {'x': current_state['x'] + 0.0, 'y': current_state['y'] + 2.0, 'z': current_state['z'] - 1.5, 'yaw':3.0},
                

                
            ]
            first_waypoint = True # True means the the first one is not taken into consideration
            counter = 0
            flag = False
            
            for waypoint in waypoints:
                rov_pos = -3.5
                dp_point = False
                
                if counter in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15, 16]:
                    rov_pos = -2.5
                if counter in [0 ,1, 2, 3, 4, 5, 6]:
                    dp_point = True
                    
                counter = counter + 1
                self.send_position_request(waypoint['x'], waypoint['y'], -waypoint['z'], waypoint['yaw'])
                while True:
                    current_position = self.fetch_current_state_local()
                    if self.has_reached_position(current_position, waypoint):
                        self.get_logger().info(f"Reached waypoint: {waypoint}")
                        first_waypoint = False
                        break
                        sleep(0.5)
                    if not first_waypoint and dp_point == False: 
                        if self.has_moved_significantly(current_position):
                            
                            adjusted_position = {
                                    'x': current_position['x'] + 0.0,
                                    'y': current_position['y'] + rov_pos,
                                    'z': current_position['z'] + 0.0,
                                    'yaw': current_position['yaw'] + 0.0
                                }
                            self.publish_waypoint(adjusted_position)
            
            # for waypoint in waypoints:
            #     counter = counter + 1
            #     flag = True
            #     if True: #(3 < counter < 9):
            #         flag = True
            #     self.send_position_request(waypoint['x'], waypoint['y'], -waypoint['z'], waypoint['yaw'])
            #     while True:
            #         current_position = self.fetch_current_state_local()
            #         if self.has_reached_position(current_position, waypoint):
            #             self.get_logger().info(f"Reached waypoint: {waypoint}")
            #             first_waypoint = False
            #             break
            #             sleep(1.0)
            #         if not first_waypoint or flag: 
            #             if self.has_moved_significantly(current_position):
                            
            #                 adjusted_position = {
            #                         'x': current_position['x'] + 0.0,
            #                         'y': current_position['y'] + rov_pos,
            #                         'z': current_position['z'] + 0.0,
            #                         'yaw': current_position['yaw'] + 0.0
            #                     }
            #                 self.publish_waypoint(adjusted_position)
                            #self.publish_waypoint(current_position)
                
            #self.set_vehicle_mode('ALT_HOLD')
            #GUIDED
            self.set_vehicle_mode('GUIDED')
            self.get_logger().info("Vehicle switched to ALT_HOLD mode")

            current_position = self.fetch_current_state_local()
            self.get_logger().info(f"Final Position: x={current_position['x']}, y={current_position['y']}, z={current_position['z']}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self.disarm_vehicle()

def main(args=None):
    rclpy.init(args=args)
    mavlink_node = MavlinkControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(mavlink_node)

    try:
        mavlink_node.get_logger().info('MavlinkControlNode node is running')
        executor.spin()
    except KeyboardInterrupt:
        mavlink_node.get_logger().info('MavlinkControlNode node is shutting down')
    finally:
        mavlink_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()