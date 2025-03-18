from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动发布者节点
        Node(
            package='topic_demo',
            executable='publisher',
            name='publisher_node',
            output='screen'
        ),
        # 启动订阅者节点
        Node(
            package='topic_demo',
            executable='subscriber',
            name='subscriber_node',
            output='screen'
        )
    ])