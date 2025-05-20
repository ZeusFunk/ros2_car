import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from my_interfaces.srv import MoveToPosition
from turtlesim.srv import TeleportAbsolute  
from geometry_msgs.msg import Twist  # 用于发布速度指令

# TurtleBotServer类：实现了一个ROS2服务端节点，接收目标位置请求并控制turtlesim小海龟移动。
class TurtleBotServer(Node):

    def __init__(self, name):
        super().__init__(name)
        # 订阅turtlesim的位姿信息
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # 发布速度指令到turtlesim
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Service is on work')
        # 创建自定义服务，响应move_to_position请求
        self.srv = self.create_service(MoveToPosition, 'move_to_position', self.move_to_position_callback)
        # 当前位姿初始化
        self.current_pose = Pose()

    def pose_callback(self, msg):
        # 回调函数：更新当前位姿
        self.current_pose = msg  

    def move_to_position_callback(self, request, response):
        # 服务回调：接收目标位置请求，发布速度指令
        target_x = request.x
        target_y = request.y
        msg = Twist()
        # 这里直接将目标位置赋值给速度，实际应用应实现更复杂的控制逻辑
        msg.linear.x = target_x  # 设置线速度 (m/s)
        msg.angular.z = target_y  # 设置角速度 (rad/s)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing velocity: linear={msg.linear.x}, angular={msg.angular.z}')
        return response

# main函数：节点入口，启动服务端

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotServer("service_server")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
