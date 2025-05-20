import sys
import rclpy
from rclpy.node import Node
from my_interfaces.srv import MoveToPosition

# TurtleBotClient类：实现了一个ROS2客户端节点，用于向服务端发送目标位置请求。
class TurtleBotClient(Node):

    def __init__(self, name):
        super().__init__(name)
        # 创建服务客户端，连接move_to_position服务
        self.cli = self.create_client(MoveToPosition, 'move_to_position')
        # 等待服务端可用，增强鲁棒性
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveToPosition.Request()
        self.future = None  # 初始化future，便于后续复用

    def send_request(self, x, y):
        # 发送目标位置请求，参数校验增强鲁棒性
        try:
            self.req.x = float(x)
            self.req.y = float(y)
        except ValueError:
            self.get_logger().error('Invalid input: x and y must be float values')
            return None
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

# main函数：节点入口，解析参数并发送请求

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotClient("service_client")
    # 参数校验，提升健壮性
    if len(sys.argv) < 3:
        node.get_logger().error('Usage: ros2 run <package> <executable> <x> <y>')
        rclpy.shutdown()
        return
    response = node.send_request(sys.argv[1], sys.argv[2])
    if response is None:
        node.get_logger().error('Request failed due to invalid input.')
        node.destroy_node()
        rclpy.shutdown()
        return
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future and node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    f'Service call failed {e!r}')
            else:
                node.get_logger().info('OK')
            break
    node.destroy_node()
    rclpy.shutdown()
