import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import droneProperties as dp

class centralController(Node):
 
    PUBLISH_RATE_HZ = 1.0  # How often to send a test message
 
    def __init__(self):
        super().__init__("central_controller")
 
        self.get_logger().info("Central Controller starting...")
 
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
        for i in range(3):
            self.droneProperties.append(dp.DroneProperties(f"drone_{i}"))
 
 
 

    def control_loop(self):
        # This is the main control loop, called at a fixed rate by the timer
        
        self.testPublish()
        
    
    
    def testPublish(self):
        
        msg = String()
        msg.data = f"ALL|HELLO tick={self.tickNum}"
        self.cmd_pubhlisher.publish(msg)
        self.tickNum += 1
 

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