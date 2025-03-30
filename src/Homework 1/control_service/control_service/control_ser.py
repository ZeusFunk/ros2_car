import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from my_interfaces.srv import MoveToPosition
from turtlesim.srv import TeleportAbsolute  
from geometry_msgs.msg import Twist  # 用于发布速度指令

class TurtleBotServer(Node):

    def __init__(self, name):
        super().__init__(name)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Service is on work')
        self.srv = self.create_service(MoveToPosition, 'move_to_position', self.move_to_position_callback)
        self.current_pose = Pose()


    def pose_callback(self, msg):
        self.current_pose = msg  

    def move_to_position_callback(self, request, response):
        target_x = request.x
        target_y = request.y
        msg = Twist()
        msg.linear.x = target_x  # 设置线速度 (m/s)
        msg.angular.z = target_y  # 设置角速度 (rad/s)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing velocity: linear={msg.linear.x}, angular={msg.angular.z}')
        return response



def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotServer("service_server")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
