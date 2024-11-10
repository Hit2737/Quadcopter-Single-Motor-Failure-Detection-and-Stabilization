# motor_model.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ThrottlePublisher(Node):
    def __init__(self):
        super().__init__('throttle_publisher')
        self.throttle_values = [0.727, 0.727, 0.727, 0.727]
        self.publisher_ = self.create_publisher(Float32MultiArray, 'throttle_values', 10)
        self.timer = self.create_timer(0.005, self.publish_throttle_values)

    def publish_throttle_values(self):
        msg = Float32MultiArray()
        # Replace with your throttle values
        msg.data = self.throttle_values
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published throttle values: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ThrottlePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()