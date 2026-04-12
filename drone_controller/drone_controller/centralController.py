

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from helpers.droneProperties import droneProperties as dp

class centralController(Node):
 
    PUBLISH_RATE_HZ = 1.0 
 
    def __init__(self):
        super().__init__("central_controller")

       
 
        self.get_logger().info("Central Controller starting...t")
        
        self.declare_parameter("num_drones", 0)
        self.numDrones = self.get_parameter("num_drones").value
 
        self.cmd_pubhlisher = self.create_publisher(
            String,
            "/coordinator/commands",
            10,
        )

        self._timer = self.create_timer(
            1.0 / self.PUBLISH_RATE_HZ,
            self.control_loop,
        )
 
        self.tickNum = 0
        self.get_logger().info("Central Controller ready. Publishing on /coordinator/commands")
        
        self.droneProperties = []
        for i in range(self.numDrones):
            self.droneProperties.append(dp(f"drone_{i}"))
            self.get_logger().info("added drone")

        self.dronSub = self.create_subscription(
            String,
            "/drones/state",
            self.droneStateCallback,
            10,
        )
 
 
    def droneStateCallback(self, msg):
        
        #self.get_logger().info(f"Received drone state: {msg.data}")

        self.updateStates(msg)

    def updateStates(self, msg):
        if msg.data is None or "|" not in msg.data:
            return
        num = None
        
        try:
            num = str(msg.data).split(" | ")[0] # I will just structure the messages as [drone_num] | [state info]
            msg = str(str(msg.data).split(" | ")[1])
        except Exception as e:
            self.get_logger().warn(f"Received malformed state message: {msg.data} - error: {e}")
            return
        
        self.get_logger().info(f"Updating state for drone {num}: {msg}")

        if "pos" in msg:
            # I will just structure the message as [drone_num]|pos: x y z
            pos_str = msg.split("pos: ")[1]
            pos_list = pos_str.split(" ")
            x = float(pos_list[0])
            y = float(pos_list[1])
            z = float(pos_list[2])
            self.droneProperties[int(num)].set_position(x, y, z)
            self.get_logger().info(f"Updated position for drone {num} to [{x}, {y}, {z}]")
            self.get_logger().info(f"Drone {num} is now at position: {self.droneProperties[int(num)].get_position()}")
        elif "ready" in msg:
            self.droneProperties[int(num)].set_status("ready")
        elif "moving" in msg:
            self.droneProperties[int(num)].set_status("moving")
        else:
            self.get_logger().warn(f"Received unknown state info for drone {num}: {msg}")
            self.droneProperties[int(num)].set_status("unknown")
        time.sleep(0.2)

        
        


    def control_loop(self):
        # This is the main control loop, called at a fixed rate by the timer
        self.tickNum += 1
        if self.tickNum % 10 == 0:  # Every 10 ticks (10 seconds at 1 Hz)
            self.get_logger().info(f"Running control loop... {self.tickNum}")
        
        for d in self.droneProperties:
            if d.get_status() == "ready":
                if d.get_position() == [0.0, 0.0, 3.0]:
                    
                    msg = String()
                    msg.data = f"{d.get_id().split('_')[1]}|GOTO: x:5.0 y:5.0 z:3.0"
                    self.cmd_pubhlisher.publish(msg)
                    self.get_logger().info(f"Sent GOTO command to {d.get_id()} to move to (5.0, 5.0, 3.0)")
                else:
                    self.get_logger().info(f"Drone {d.get_id()} is at the position. {d.get_position()}")
                    msg = String()
                    msg.data = f"{d.get_id().split('_')[1]}|GOTO: x:0.0 y:0.0 z:3.0"
                    self.cmd_pubhlisher.publish(msg)
                    self.get_logger().info(f"Sent GOTO command to {d.get_id()} to move to (0.0, 0.0, 3.0)")
    
    
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