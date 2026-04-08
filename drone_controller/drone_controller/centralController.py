import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from helpers.droneProperties import droneProperties as dp

class centralController(Node):
 
    PUBLISH_RATE_HZ = 1.0  # How often to send a test message
 
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
 
 
 

    def control_loop(self):
        # This is the main control loop, called at a fixed rate by the timer
        
        self.testPublish()
        
    
    
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