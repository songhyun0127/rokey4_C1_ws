import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',  # 퍼블리셔와 동일한 토픽 이름
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
