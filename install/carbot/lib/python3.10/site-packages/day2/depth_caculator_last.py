import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthRGBQueryNode(Node):
    def __init__(self, x, y):
        super().__init__('depth_query_node')
        self.x = x
        self.y = y
        self.bridge = CvBridge()
        self.depth_image = None
        self.rgb_image = None

        # Depth 이미지 구독
        self.depth_sub = self.create_subscription(
            Image,
            '/robot8/oakd/stereo/image_raw',
            self.depth_callback,
            10)

        # RGB 이미지 구독
        self.rgb_sub = self.create_subscription(
            Image,
            '/robot8/oakd/rgb/image_raw',
            self.rgb_callback,
            10)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.check_and_print()

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.check_and_print()

    def check_and_print(self):
        if self.depth_image is not None and self.rgb_image is not None:
            h, w = self.depth_image.shape[:2]
            if 0 <= self.x < w and 0 <= self.y < h:
                depth_value = self.depth_image[self.y, self.x]
                rgb_value = self.rgb_image[self.y, self.x]  # BGR 값

                # 깊이 단위 계산 (mm 또는 m)
                if self.depth_image.dtype == np.uint16:
                    depth_in_m = depth_value / 1000.0
                else:
                    depth_in_m = depth_value

                self.get_logger().info(
                    f"좌표 ({self.x},{self.y}) → 깊이: {depth_in_m:.3f} m | RGB (BGR): {rgb_value}"
                )
            else:
                self.get_logger().warn("입력한 좌표가 이미지 범위를 벗어났습니다.")

            rclpy.shutdown()  # 한 번 계산 후 종료

def main():
    rclpy.init()
    x = int(input("깊이 확인할 x 좌표 입력: "))
    y = int(input("깊이 확인할 y 좌표 입력: "))
    node = DepthRGBQueryNode(x, y)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
