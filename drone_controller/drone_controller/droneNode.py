#!/usr/bin/env python3
from multiprocessing import connection
import sys
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
        
        self.initialAltitude = None
        
        self.cmd_msg = String()
        
        self.debugLoop = 0
        
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
            
        self.get_logger().info('Drone Node connected and listening.')

    def control_loop(self):
        # This is the main control loop, called at a fixed rate by the timer
        # self.get_logger().info(f"Current state: {self.vehicle.system_status.state}")
        self.get_logger().info("Control loop tick...")
        self.debugLoop += 1
        self.get_logger().info(f"Debug loop count: {self.debugLoop}")
        
        if not self.takenOff:
            self.takenOff = self.arm_and_takeoff(2)
        if self.takenOff:
            pass
        
        altMsg = self.vehicle.recv_match(type='VFR_HUD', blocking=False)
        if altMsg:
            self.get_logger().info(f"Current Altitude: {altMsg.alt:.2f}m")
        
        
        
        #self.get_logger().info(self.vehicle.get_vehicle_status())
        
        #self.get_logger().info(f'I heard: "{msg.data}"')
        # if not self.takenOff:
        #     self.get_logger().info("Not taken off yet")
        # if self.takenOff:
        #     if self.waypoints == 0:
        #         self.goto_position(5, 0, 2)
        #         self.waypoints += 1
        #     elif self.waypoints == 1:
        #         self.goto_position(5, 5, 2)
        #         self.waypoints += 1
        #     elif self.waypoints == 2:
        #         self.goto_position(0, 5, 2)
        #         self.waypoints += 1
        #     elif self.waypoints == 3:
        #         self.goto_position(0, 0, 2)
        #         self.waypoints += 1
        #     else:
        #         self.get_logger().info("All waypoints reached.")
        




    def listener_callback(self, msg):
        
        self.cmd_msg = msg


    def goto_position(self, x, y, z_alt):
        # 1. Set Mode to GUIDED
        # This tells ArduPilot to listen to external ROS/MAVLink commands
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4) # 4 is the constant for 'GUIDED' in ArduPilot Copter

        # 2. Check for Arming
        if not self.vehicle.motors_armed():
            self.get_logger().info("Arming motors...")
            self.vehicle.arducopter_arm()
            self.vehicle.motors_armed_wait()

        # 3. Send the Movement Command
        # We use SET_POSITION_TARGET_LOCAL_NED
        self.get_logger().info(f"Targeting X: {x}, Y: {y}")
        self.vehicle.mav.set_position_target_local_ned_send(
            0,                                  # time_boot_ms
            self.vehicle.target_system, 
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Use the Gazebo/SITL coordinate frame
            0b110111111000,                     # Bitmask (tells ArduPilot to only look at XYZ)
            x, y, -z_alt,                       # Positions (Z is negative for altitude)
            0, 0, 0,                            # Velocities (ignored by bitmask)
            0, 0, 0,                            # Accelerations (ignored by bitmask)
            0, 0)                               # Yaw (ignored by bitmask)
        
        
    def check_prearm_msgs(self):
        
        armMsg = self.vehicle.recv_match(type='SYS_STATUS', blocking=False)
        statusMsg = self.vehicle.recv_match(type='STATUSTEXT', blocking=False)

        
        # # Bit 0x4000000 (decimal 67108864) is the Pre-arm check bit in ArduPilot
        PREARM_BIT = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK 

        print("checking prearm msgs")
        
        if armMsg:
            print(armMsg.onboard_control_sensors_health & PREARM_BIT)
            if armMsg.onboard_control_sensors_health & PREARM_BIT:
                self.get_logger().info("Pre-arm checks passed!")
                self.prearm = True
            
        print("checking imu bits")
        
        if statusMsg:    
            if "IMU1 is using" in statusMsg.text:
                self.get_logger().info("IMU1 is using GPS for position. GPS is ready!")
                self.gpsready = True
            
                
        altMsg = self.vehicle.recv_match(type='VFR_HUD', blocking=False)
        if altMsg:
            self.get_logger().info(f"Current Altitude: {altMsg.alt:.2f}m")
            self.initialAltitude = altMsg.alt

        
        if self.prearm and self.gpsready and self.initialAltitude:
            return True
            
        

        return False
            
            
    
    def arm_and_takeoff(self, target_altitude):
        self.vehicle.wait_gps_fix()
               
        self.get_logger().info("Doing pre-arm checks...")

        while not self.check_prearm_msgs():
            self.get_logger().warn("Pre-arm checks failed. Waiting...")
            time.sleep(1)
        
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
        
        # while self.vehicle.location.__str__ < 1.5:
        #     self.get_logger().info(f"Waiting for appropriate altitude: {self.vehicle.location.alt:.2f}m")
        #     time.sleep(1)
        
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




    # node = droneNode()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     node.get_logger().info("Drone Node shutting down.")
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()