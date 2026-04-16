#!/usr/bin/env python3
from enum import Enum
from multiprocessing import connection
import sys
import time

import time
from unittest import case

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil
import numpy as np

from sensor_msgs.msg import LaserScan


class State(Enum):
    INIT = 0
    LAUNCHING = 1
    WAITING = 2
    MOVING = 3

    PAUSE = 9



class droneNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'drone_node_{drone_id}')
        self.name = f'drone_{drone_id}'
        self.id = drone_id
        
        self.state = State.INIT
        self.lastState = State.INIT
        self.prearm = False
        self.gpsready = False

        self.pose = [0.0, 0.0, 0.0]  # (latitude, longitude, altitude)

        self.ready = False

        if drone_id == 0:
            self.ready = True

        self.instruction = None


        self.msg_cache = {}
        
        self.initialAltitude = None

        self.busy = False
        
        self.cmd_msg = String()
        
        self.debugLoop = 0

        self.startTime = time.time()

        self.goneToStart = False

        self.destination = (0.0, 0.0, 0.0)
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
        
      
        
        self.cmd_sub = self.create_subscription(
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
            'drones/state',
            10)
        
        self.subPrev = self.create_subscription(
            String,
            'drones/state',
            self.prev_callback,
            10)
        
        self.subLidar = self.create_subscription(
            LaserScan,
            f'/iris_{self.id}/scan',
            self.subLidar_callback,
            10)

            
        self.get_logger().info('Drone Node connected and listening.')

    def prev_callback(self, msg):
        pass

    def subLidar_callback(self, msg):
        
        front = msg.ranges[0]
        front_left = msg.ranges[45]
        front_right = msg.ranges[315]
        self.get_logger().info(f"Lidar readings - Front: {front:.2f}m, Front-Left: {front_left:.2f}m, Front-Right: {front_right:.2f}m")

        if front < 5.0 and self.debugLoop > 30:  # If an obstacle is closer than 1 meter in front
            self.get_logger().warn("Obstacle detected in front! Stopping movement.")
            self.vehicle.mav.set_position_target_local_ned_send(
                0,
                self.vehicle.target_system, 
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # Position only
                self.pose[0], self.pose[1], -self.pose[2],  # Hold current position
                0, 0, 0,  # Velocity (ignored)
                0, 0, 0,  # Acceleration (ignored)
                0, 0)     # Yaw (ignored)
            self.state = State.PAUSE

        return

    def control_loop(self):
        # This is the main control loop, called at a fixed rate by the timer
        # self.get_logger().info(f"Current state: {self.vehicle.system_status.state}")
        #self.get_logger().info("Control loop tick...")
        self.debugLoop += 1
        #self.get_logger().info(f"Debug loop count: {self.debugLoop}")
        self.get_logger().info(f"Current state: {self.state.name}") 
        
        

        
        # if self.debugLoop < 30:
        #     self.goto(5, 0, 7)

        if self.debugLoop % 10 == 0: 
            self.do_logs()

    
        match self.state:
            case State.INIT:
                if not self.takenOff:
                    self.get_logger().info("Initiating takeoff sequence...")
                    
                    self.takenOff = self.arm_and_takeoff(1)
                    self.get_logger().info(f"Takeoff status: {'Success' if self.takenOff else 'Failed'}")
                    self.state = State.LAUNCHING
                    time.sleep(0.2)
                if self.takenOff:
                    pass


            case State.LAUNCHING:
                if not self.goneToStart:
      
                    self.gotoStart()
                    self.goneToStart = True

                pass
            case State.WAITING:
                self.goto(self.pose[0] + 10, self.pose[1], 7.5)


                pass
            case State.MOVING:
                self.get_logger().info(f"Moving towards destination: {self.destination}")
                self.moving()

            case State.PAUSE:
                pass

            
            case _:


                pass

        
    def gotoStart(self):
        self.goto(-4.5, 0, 7.5)

    def moving(self):
        x,y,z = self.destination
        arrived = False
        try:
            msg = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
            if msg:
                #self.pubState.publish(String(data=f"{self.id} | pos: {msg.x} {msg.y} {-msg.z}"))
                self.pose = [msg.x, msg.y, -msg.z]
                if abs(msg.x - x) < 0.2 and abs(msg.y - y) < 0.2 and abs(-msg.z - z) < 0.2:

                    arrived = True
        except Exception as e:
            self.get_logger().error(f"Error receiving position: {e}")

        self.pubState.publish(String(data=f"{self.id} | pos: {x} {y} {z}"))
        if arrived:
            self.get_logger().info(f"Arrived at destination! {x}, {y}, {z}")
            time.sleep(2)  # Small delay to stabilize at the position
            self.state = State.WAITING
            self.pubState.publish(String(data=f"{self.id} | ready"))



    def goto(self, x, y, z):
        self.lastState = self.state
        
        self.destination = (x, y, z)
        self.get_logger().info(f"Going to position: ({x}, {y}, {z})")
        
        self.vehicle.mav.set_position_target_local_ned_send(
            0,                                               # Time boot ms (not used)
            self.vehicle.target_system, 
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,            # Use the local coordinate system
            0b0000111111111000,                             # Bitmask: only use Pos X, Y, Z
            x, y, -z,                                 # X, Y, Z (Z is negative for altitude!)
            0.2, 0.2, 0.2,                                        # Velocity X, Y, Z 
            0, 0, 0,                                        # Acceleration (ignored)
            0, 0                                            # Yaw, Yaw rate (ignored)
        )
        self.pubState.publish(String(data=f"{self.id} | moving"))
        self.state = State.MOVING
        
        


    def do_logs(self):
        try:
            altMsg = self.vehicle.recv_match(type='VFR_HUD', blocking=False)
            if altMsg:
                self.get_logger().info(f"Current Altitude: {altMsg.alt - self.initialAltitude:.2f}m")
            self.get_logger().info(f"current tick: {self.debugLoop}")
        except Exception as e:
            self.get_logger().error(f"Error in do_logs: {e}")


    def listener_callback(self, msg):
        if self.debugLoop % 10 == 0: 
            self.get_logger().info("listener callback")
            self.get_logger().info(f"Received command: {msg.data} at start of listener callback")
        if str(msg.data)[0] != str(self.id):
            return  # Ignore messages not intended for this drone
        self.cmd_msg = msg
        self.get_logger().info(f"Received command: {msg.data}")
        if "GOTO" in msg.data and False:
            # Expecting format: "drone_0|GOTO: x:5.0 y:5.0 z:3.0"
            try:
                cmd_parts = msg.data.split("|")[1].split("GOTO: ")[1]
                coords = cmd_parts.split(" ")
                x = float(coords[0].split(":")[1])
                y = float(coords[1].split(":")[1])
                z = float(coords[2].split(":")[1])
                self.goto(x, y, z)
            except Exception as e:
                self.get_logger().error(f"Failed to parse GOTO command: {e}")
        return





        
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
            self.get_logger().info(f"Current Altitude: {msg.alt:.2f}m")
            self.initialAltitude = msg.alt
        
        if self.prearm and self.gpsready and self.initialAltitude or time.time() - self.startTime > 180: 
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
            time.sleep(0.2)
        self.get_logger().info(f"published initial position: {self.id} | pos: 0.0 0.0 {target_altitude}")
        self.pubState.publish(String(data=f"{self.id} | pos: 0.0 0.0 {target_altitude}"))



     
 
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