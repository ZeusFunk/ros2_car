import sys
import rclpy
from rclpy.node import Node
from my_interfaces.srv import MoveToPosition

class TurtleBotClient(Node):

    def __init__(self,name):
        super().__init__(name)
        self.cli = self.create_client(MoveToPosition, 'move_to_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveToPosition.Request()

    def send_request(self, x, y):
        self.req.x = x
        self.req.y = y
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotClient("service_client")
    node.send_request(float(sys.argv[1]), float(sys.argv[2]))
    while rclpy.ok():                                                             # ROS2系统正常运行
        rclpy.spin_once(node)
        if node.future.done():                                                    # 数据是否处理完成
            try:
                response = node.future.result()                                   # 接收服务器端的反馈数据
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(                                           # 将收到的反馈信息打印输出
                    'OK')
            break
            
    node.destroy_node()   
    rclpy.shutdown()
