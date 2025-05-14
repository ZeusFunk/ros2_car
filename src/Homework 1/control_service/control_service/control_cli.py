'''
Author: Jungle jiangyijun2003@outlook.com
Date: 2025-05-14 16:03:25
LastEditors: Jungle jiangyijun2003@outlook.com
LastEditTime: 2025-05-14 16:06:43
FilePath: \ros2_car\src\Homework 1\control_service\control_service\control_cli.py
Description: ROS2 客户端节点，用于向服务发送请求并接收响应。

Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
'''

# 导入必要的库
import sys
import rclpy  # ROS2 Python客户端库
from rclpy.node import Node  # ROS2节点基类
from my_interfaces.srv import MoveToPosition  # 自定义服务接口

# 定义一个客户端节点类，用于与服务通信
class TurtleBotClient(Node):

    def __init__(self, name):
        # 初始化父类Node
        super().__init__(name)
        # 创建一个客户端，连接到名为'move_to_position'的服务
        self.cli = self.create_client(MoveToPosition, 'move_to_position')
        # 等待服务可用
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # 创建一个服务请求对象
        self.req = MoveToPosition.Request()

    # 发送请求到服务端
    def send_request(self, x, y):
        # 设置请求参数
        self.req.x = x
        self.req.y = y
        # 异步调用服务
        self.future = self.cli.call_async(self.req)
        # 等待服务响应完成
        rclpy.spin_until_future_complete(self, self.future)
        # 返回服务响应结果
        return self.future.result()

# 主函数
def main(args=None):
    # 初始化ROS2客户端库
    rclpy.init(args=args)
    # 创建客户端节点实例
    node = TurtleBotClient("service_client")
    # 发送请求，参数从命令行获取
    node.send_request(float(sys.argv[1]), float(sys.argv[2]))
    # 循环监听服务响应
    while rclpy.ok():  # ROS2系统正常运行
        rclpy.spin_once(node)  # 处理一次事件循环
        if node.future.done():  # 检查服务响应是否完成
            try:
                # 获取服务响应结果
                response = node.future.result()
            except Exception as e:
                # 如果调用失败，打印错误信息
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # 如果调用成功，打印响应信息
                node.get_logger().info('OK')
            break  # 退出循环
            
    # 销毁节点并关闭ROS2客户端库
    node.destroy_node()   
    rclpy.shutdown()
