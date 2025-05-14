import rclpy  # ROS 2 Python客户端库
from rclpy.node import Node  # ROS 2节点基类
from turtlesim.msg import Pose  # 用于接收乌龟的位姿信息
from my_interfaces.srv import MoveToPosition  # 自定义服务接口，用于移动到指定位置
from turtlesim.srv import TeleportAbsolute  # 用于绝对位置传送（未使用）
from geometry_msgs.msg import Twist  # 用于发布速度指令

class TurtleBotServer(Node):
    """
    一个ROS 2服务节点，用于控制Turtlesim中的乌龟移动。
    """

    def __init__(self, name):
        # 初始化父类Node
        super().__init__(name)
        
        # 创建订阅者，订阅乌龟的位姿信息
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 创建发布者，用于发布速度指令
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 日志信息，表示服务已启动
        self.get_logger().info('Service is on work')
        
        # 创建服务，处理移动到指定位置的请求
        self.srv = self.create_service(MoveToPosition, 'move_to_position', self.move_to_position_callback)
        
        # 当前乌龟的位姿
        self.current_pose = Pose()

    def pose_callback(self, msg):
        """
        订阅回调函数，用于更新当前乌龟的位姿。
        """
        self.current_pose = msg  # 更新当前位姿

    def move_to_position_callback(self, request, response):
        """
        服务回调函数，处理移动到指定位置的请求。
        """
        # 从请求中获取目标位置
        target_x = request.x
        target_y = request.y
        
        # 创建速度消息
        msg = Twist()
        msg.linear.x = target_x  # 设置线速度 (m/s)
        msg.angular.z = target_y  # 设置角速度 (rad/s)
        
        # 发布速度指令
        self.publisher.publish(msg)
        
        # 日志信息，显示发布的速度
        self.get_logger().info(f'Publishing velocity: linear={msg.linear.x}, angular={msg.angular.z}')
        
        return response  # 返回响应

def main(args=None):
    """
    主函数，初始化节点并运行。
    """
    rclpy.init(args=args)  # 初始化rclpy
    node = TurtleBotServer("service_server")  # 创建节点
    rclpy.spin(node)  # 保持节点运行
    node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy

if __name__ == '__main__':
    main()  # 运行主函数
