#!/usr/bin/env python3
from multiprocessing import connection
import sys
import time

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil

class droneNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'drone_node_{drone_id}')
        self.name = f'drone_{drone_id}'
        
        self.state = "INIT"
        self.prearm = False
        self.gpsready = False

        self.ready = False

        if drone_id == 0:
            self.ready = True


        self.msg_cache = {}
        
        self.initialAltitude = None

        self.busy = False
        
        self.cmd_msg = String()
        
        self.debugLoop = 0

        self.startTime = time.time()
        
        # MAVProxy creates UDP outputs starting at 14550. 
        # Let's use 14551 for Drone 0, 14561 for Drone 1, etc.
        self.mav_port = 14551 + (drone_id * 10)
        
        
        self.get_logger().info(f"Connecting to {self.name} on UDP Port {self.mav_port}...")
        
        # Use 'udpin' because MAVProxy is pushing data TO this port
        self.connection_string = f'udpin:127.0.0.1:{self.mav_port}'
        self.vehicle = mavutil.mavlink_connection(self.connection_string)
        
        self.get_logger().info(f"Waiting for heartbeat from the drone on {self.connection_string}...")
        
        self.vehicle.wait_heartbeat()
        self.get_logger().info("Heartbeat received!")
        
        

        self.takenOff = False
        self.waypoints = 0
        
      
        
        self.sub = self.create_subscription(
            String, 
            '/coordinator/commands', 
            self.listener_callback, 
            10)
        
        self._timer = self.create_timer(
            1.0,  # 1 second
            self.control_loop,
        )

        self.pubState = self.create_publisher(
            String,
            'drone' + self.name + '/state',
            10)
        
        self.subPrev = self.create_subscription(
            String,
            'drone' + (str(drone_id - 1) if drone_id > 0 else '0') + '/state',
            self.prev_callback,
            10)
            
        self.get_logger().info('Drone Node connected and listening.')

    def prev_callback(self, msg):
        if msg.data == "1":
            self.get_logger().info(f"Received ready signal from previous drone. This drone is now ready.")
            self.ready = True

    def control_loop(self):
        # This is the main control loop, called at a fixed rate by the timer
        # self.get_logger().info(f"Current state: {self.vehicle.system_status.state}")
        #self.get_logger().info("Control loop tick...")
        self.debugLoop += 1
        #self.get_logger().info(f"Debug loop count: {self.debugLoop}")
        
        if not self.takenOff:
            self.takenOff = self.arm_and_takeoff(2)
            self.get_logger().info(f"Takeoff status: {'Success' if self.takenOff else 'Failed'}")
            self.pubState.publish(String(data="1"))  # Publish "1" to indicate takeoff complete
        if self.takenOff:
            pass

        if self.debugLoop % 10 == 0: 
            self.do_logs()


        if not self.busy: # will have to work something out for this as it isn't a blocking function to send it somewhere, so will need to establish arrival
            self.get_logger().warn("Not busy. Sending a waypoint command to move to (5, 5, 3)")
            self.busy = True
            
            # To fly to X=5, Y=5, Z=3 (relative to where it started)
            self.vehicle.mav.set_position_target_local_ned_send(
                0,                                               # Time boot ms (not used)
                self.vehicle.target_system, 
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,            # Use the local coordinate system
                0b0000111111111000,                             # Bitmask: only use Pos X, Y, Z
                5.0, 5.0, -3.0,                                 # X, Y, Z (Z is negative for altitude!)
                0, 0, 0,                                        # Velocity X, Y, Z (ignored)
                0, 0, 0,                                        # Acceleration (ignored)
                0, 0                                            # Yaw, Yaw rate (ignored)
            )
            self.busy = False



    def do_logs(self):
        
        altMsg = self.vehicle.recv_match(type='VFR_HUD', blocking=False)
        if altMsg:
            self.get_logger().info(f"Current Altitude: {altMsg.alt - self.initialAltitude:.2f}m")
        self.get_logger().info(f"current tick: {self.debugLoop}")


    def listener_callback(self, msg):
        
        self.cmd_msg = msg



        
    def check_prearm_msgs(self):
        

        # # Bit 0x4000000 (decimal 67108864) is the Pre-arm check bit in ArduPilot
        PREARM_BIT = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK 
        
        msg = self.vehicle.recv_match(type=['SYS_STATUS', 'STATUSTEXT', 'VFR_HUD'], blocking=False)
        # if not "None" in str(msg):
        #     self.get_logger().info(f"Received message: {msg}")
        #     print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

        if 'SYS_STATUS' in str(msg) and self.prearm == False:
            if msg.onboard_control_sensors_health & PREARM_BIT:
                self.get_logger().info("Pre-arm checks passed!")
                self.prearm = True
        
        if 'STATUSTEXT' in str(msg) and self.gpsready == False:
            if "is using GPS" in msg.text:
                self.get_logger().info("IMU is using GPS for position. GPS is ready!")
                self.gpsready = True    
        
        if 'VFR_HUD' in str(msg) and self.initialAltitude is None:
            #self.get_logger().info(f"Current Altitude: {msg.alt:.2f}m")
            self.initialAltitude = msg.alt
        
        if self.prearm and self.gpsready and self.initialAltitude or time.time() - self.startTime > 90: 
            return True
            
        

        return False

    
    def arm_and_takeoff(self, target_altitude):
        self.vehicle.wait_gps_fix()
               
        self.get_logger().info("Doing pre-arm checks...")
        self.get_logger().warn("Pre-arm checks failed. Waiting...")
        while not self.check_prearm_msgs():
            
            pass
        
        time.sleep(0.5)  # Small delay before arming
        
        self.get_logger().info("Arming and taking off...")
        
        try:
            self.vehicle.set_mode("GUIDED")
            self.get_logger().info("Mode set to GUIDED. Arming motors...")
            time.sleep(2)


                
            self.vehicle.arducopter_arm()
            self.vehicle.motors_armed_wait()
            
            time.sleep(2)
            
            self.get_logger().info("Motors armed. Sending takeoff command...")
            
            # Send takeoff command
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system, self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, target_altitude)
        except Exception as e:
            self.get_logger().error(f"Error during takeoff: {e}")
            return False
        self.get_logger().info(f"Taking off to {target_altitude} meters...")
        
        


        altMsg = self.vehicle.recv_match(type='VFR_HUD', blocking=False)
        if altMsg:
            self.get_logger().info(f"Current Altitude: {altMsg.alt - self.initialAltitude:.2f}m")


        while altMsg.alt - self.initialAltitude < target_altitude * 0.95:
            altMsg = self.vehicle.recv_match(type='VFR_HUD', blocking=True)
            if altMsg:
                self.get_logger().info(f"Current Altitude: {altMsg.alt - self.initialAltitude:.2f}m")

        self.busy = False
 
        return True

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run drone_controller droneNode [drone_id]")
        return

    try:
        drone_id = int(sys.argv[1])
        node = droneNode(drone_id)
        rclpy.spin(node)
    except ValueError:
        print("Error: Drone ID must be an integer.")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()