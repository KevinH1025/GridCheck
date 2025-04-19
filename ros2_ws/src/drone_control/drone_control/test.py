import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class RelAltTest(Node):
    def __init__(self):
        super().__init__('rel_alt_test')

        # Very important to set the communication process 
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.callback,
            qos
        )
        self.get_logger().info("✅ Subscribed to /mavros/global_position/rel_alt")

    def callback(self, msg):
        self.get_logger().info(f"[CALLBACK TRIGGERED] rel_alt: {msg.data:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RelAltTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
