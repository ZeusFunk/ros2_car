# publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 1.0  # 1秒周期
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.temperature = 20.0

    def timer_callback(self):
        # 模拟温度变化
        self.temperature += random.uniform(-1.0, 1.0)
        msg = Float32()
        msg.data = self.temperature
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    temperature_publisher = MinimalPublisher()
    rclpy.spin(temperature_publisher)
    temperature_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()