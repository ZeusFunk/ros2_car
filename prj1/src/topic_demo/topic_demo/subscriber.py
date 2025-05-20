# subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告

    def listener_callback(self, msg):
        self.get_logger().info(f'温度: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    temperature_subscriber  = MinimalSubscriber()
    rclpy.spin(temperature_subscriber)
    temperature_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()