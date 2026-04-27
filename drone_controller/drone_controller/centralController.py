
import math 
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from helpers.droneProperties import droneProperties as dp

class centralController(Node):
 
    PUBLISH_RATE_HZ = 1.0 
 
    def __init__(self):
        super().__init__("central_controller")

        
 
        self.get_logger().info("Central Controller starting...")
        
        self.declare_parameter("num_drones", 0)
        self.numDrones = self.get_parameter("num_drones").value

        self.activeDrone = 0

        self.nodes = [[-6, 0, 7, 0]] # starting node, entrance to cave is here

        self.usedWaypoints = [] # This will store the paths that have been taken through the tunnel, so that we can avoid them in the future and find new paths

        self.tickNum = 0

        self.newGaps = ""

        self.nextNodeLocation = None

        self.paths = [] # This will store the paths that can be taken - when new paths are found, they will be added to this list, and when paths are taken, they will be moved to the usedPaths list
        self.usedPaths = [] # This will store the paths that have been taken through the tunnel, so that we can avoid them in the future and find new paths
        
        
        self.droneProperties = []
        for i in range(self.numDrones):
            self.droneProperties.append(dp(f"drone_{i}"))
            self.get_logger().info("added drone")




 
        self.cmd_pubhlisher = self.create_publisher(
            String,
            "/coordinator/commands",
            10,
        )

        self._timer = self.create_timer(
            1.0 / self.PUBLISH_RATE_HZ,
            self.control_loop,
        )
 
        

        self.dronSub = self.create_subscription(
            String,
            "/drones/state",
            self.droneStateCallback,
            10,
        )




        self.get_logger().info("Central Controller ready. Publishing on /coordinator/commands")
 
    def droneStateCallback(self, msg):
        
        #self.get_logger().info(f"Received drone state: {msg.data}")

        self.updateStates(msg)

    def updateStates(self, msg):
        self.get_logger().warn("UPDATING STATES")
        if msg.data is None or "|" not in msg.data:
            return
        num = None
        
        try:
            num = str(msg.data).split(" | ")[0] # I will just structure the messages as [drone_num] | [state info]
            msg = str(str(msg.data).split(" | ")[1])
        except Exception as e:
            self.get_logger().warn(f"Received malformed state message: {msg.data} - error: {e}")
            return
        
        self.get_logger().info(f"Updating state for drone {num}: {msg}, from {self.droneProperties[self.activeDrone].get_last_status()}")

        if "pos" in msg:
            # I will just structure the message as [drone_num]|pos: x y z
            pos_str = msg.split("pos: ")[1]
            pos_list = pos_str.split(" ")
            x = float(pos_list[0]) + 5*int(num) 
            y = float(pos_list[1]) 
            z = float(pos_list[2])
            self.droneProperties[int(num)].set_position(x, y, z)
            self.get_logger().info(f"x: {x}, y: {y} z: {z}")
            # self.get_logger().info(f"Updated position for drone {num} to [{x}, {y}, {z}]")
            # self.get_logger().info(f"Drone {num} is now at position: {self.droneProperties[int(num)].get_position()}")
        elif "ready" in msg:
            self.get_logger().info(f"Drone {num} is ready")
            self.droneProperties[int(num)].set_status("ready")
        elif "moving" in msg:
            self.get_logger().info(f"Drone {num} is moving")
            self.droneProperties[int(num)].set_status("moving")
        elif "pause" in msg:
            self.get_logger().info(f"Drone {num} is paused")
            self.droneProperties[int(num)].set_status("pause")
        elif "done" in msg:
            self.get_logger().info(f"Drone {num} is done")
            self.droneProperties[int(num)].set_status("done")
        elif "GAPS" in msg:
            self.newGaps = msg.split(':')[1]
            self.get_logger().info(f"Drone {num} has found gaps: {self.newGaps}")
            self.get_logger().info("Received gaps info, creating node")

            
            self.droneProperties[int(num)].set_status("dogaps")
        elif "LAUNCHING" in msg:
            self.droneProperties[int(num)].set_status("launching")
            self.get_logger().info(f"Drone {num} is launching")

        elif "NEWNODE" in msg:
            self.doNewNodeLocation(msg)
        else:
            self.get_logger().warn(f"Received unknown state info for drone {num}: {msg}")
            self.droneProperties[int(num)].set_status("unknown")
        time.sleep(0.2)

    def doNewNodeLocation(self, msg):
        location_str = msg.split(":")[1]
        location_list = location_str.split(",")
        x = float(location_list[0])
        y = float(location_list[1])
        z = float(location_list[2])
        self.nextNodeLocation = (x, y, z)
        
    def control_loop(self):
        self.tickNum += 1
        if self.tickNum % 5 == 0: # every 5 seconds, print the status of the active drone
            self.get_logger().info(f"Current state: {self.droneProperties[self.activeDrone].get_status()} with drone number: {self.activeDrone}, with activation of: {self.droneProperties[self.activeDrone].get_activation()}")

        #if self.activeDrone == 0:
        if self.droneProperties[self.activeDrone].get_status() == "ready" and self.droneProperties[self.activeDrone].get_activation() == False:
            self.droneProperties[self.activeDrone].set_activation(True)
            self.get_logger().info(f"Drone {self.activeDrone} is ready, sending it to the start of the tunnel")
            

            sendNodes = ""
            self.get_logger().info(f"Nodes: {self.nodes}")
            for n in self.nodes:
                
                sendNodes += f"{(n[0] + (5*self.activeDrone))},{n[1]},{n[2]},{n[3]};"
                
            self.cmd_pubhlisher.publish(String(data=f"drone_{self.activeDrone}|GOTHROUGH: {sendNodes}"))
            self.get_logger().info(f"published: drone_{self.activeDrone}|GOTHROUGH: {sendNodes}")
            self.droneProperties[self.activeDrone].set_status("going_to_start")

        if self.droneProperties[self.activeDrone].get_status() == "ready" and self.droneProperties[self.activeDrone].get_last_status() == "moving":
            self.get_logger().info(f"Drone {self.activeDrone} has reached the start of the tunnel, sending it to explore")
            self.cmd_pubhlisher.publish(String(data=f"drone_{self.activeDrone}|Explore"))

        if self.droneProperties[self.activeDrone].get_status() == "ready" and self.droneProperties[self.activeDrone].get_last_status() == "going_to_start":
            self.cmd_pubhlisher.publish(String(data=f"drone_{self.activeDrone}|Explore"))
            self.get_logger().info(f"Drone {self.activeDrone} is at the start, sending it to explore")

        if self.droneProperties[self.activeDrone].get_status() == "moving":
            self.get_logger().info(f"Drone {self.activeDrone} is moving, waiting for it to finish")
            # Wait for drone 0 to finish before activating drone 1
            return
        if self.droneProperties[self.activeDrone].get_last_status() == "moving" and self.droneProperties[self.activeDrone].get_status() == "pause":
            self.get_logger().info(f"Drone {self.activeDrone} is paused, waiting for resolution")
            return
        if self.droneProperties[self.activeDrone].get_last_status() == "pause" and self.droneProperties[self.activeDrone].get_status() == "moving":
            self.get_logger().info(f"Drone {self.activeDrone} has resumed moving, waiting for it to finish")
            return
        if self.droneProperties[self.activeDrone].get_status() == "done":
            self.get_logger().info(f"Drone {self.activeDrone} has finished, activating drone {self.activeDrone + 1}")
            self.activeDrone += 1
            return

        if self.droneProperties[self.activeDrone].get_status() == "dogaps":

            self.createNode()


    def createNode(self):         

        self.get_logger().info("In create node")
        self.droneProperties[self.activeDrone].set_status("working")

        gaps = self.newGaps
        yaw = float(gaps.split("yaw-")[1])
        gaps = gaps.split("yaw")[0]

        # This function will create a new node in the tunnel and publish it to the drones
        # The drones will then use this information to navigate through the tunnel
        set = []
        # The angles for gaps are 0 forward, and increasing anticlockwise, so 90 is left, 180 is back, 270 is right. 

        g = gaps.split(",") # as it's a string for a list, it will go into  [ [[start , end] , [start , end] ... ]etc so for 3 gaps I have 6 items

        for i in range(len(g)): # this gets ride of all the extra stuff
            #self.get_logger().info(f"g[i] before: {g[i]}")
            g[i] = g[i].strip("[] ")
            #self.get_logger().info(f"g[i] after: {g[i]}")

        self.get_logger().info(f"g split and stripped: {g}")

        # for i in range(len(g)):
        #     g[i] = g[i].strip("[").strip("]") # gives just the list of numbers
        
        self.get_logger().info(f"Processed gaps: {g}")  

        for i in range(0, len(g), 2):
            middle = (float(g[i]) + float(g[i+1])) / 2
            set.append((str(-1 * math.radians(middle) + yaw)))
            self.get_logger().info(f"Added gap at angle {middle} degrees")
        x,y,z = self.nextNodeLocation
        
        
        
        
        self.get_logger().info(f"x:{x}")
        self.get_logger().info(f"y:{y}")
        self.get_logger().info(f"z:{z}")
        self.get_logger().info(f"Appended node: {[x,y,z, set[0]]}")
        
        
        #self.nodes.append([x,y,z, set[0]]) # I will just store the nodes as a list of tuples (gap angles, location) for now, and publish them to the drones as needed


        pass
    
    
    def testPublish(self):
        
        msg = String()
        for i in self.droneProperties:
            #self.get_logger().info(f"Drone {i.get_id()} is in state {i.get_status()}")
            d_num = i.get_id().split("_")[1]
            msg.data = f"drone_{d_num}|GOTO: x:{2*int(d_num)} y:0 z:2"
            self.cmd_pubhlisher.publish(msg)
        #msg.data = f"ALL|HELLO tick={self.tickNum}"
        #self.cmd_pubhlisher.publish(msg)
        self.tickNum += 1
        self.get_logger().info(f"Published a message")
 

def main(args=None):
    rclpy.init(args=args)
    node = centralController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Central Controller shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()