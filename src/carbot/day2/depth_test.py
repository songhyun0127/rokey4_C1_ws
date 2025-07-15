import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthRGBQueryNode(Node):
    def __init__(self):
        super().__init__('depth_query_node')
        self.bridge = CvBridge()
        self.depth_image = None
        self.rgb_image = None
        self.last_logged_time = self.get_clock().now()

        # 중앙 좌표 (초기값: None → 첫 이미지 수신 시 계산)
        self.x = None
        self.y = None

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
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.check_and_print()
        except Exception as e:
            self.get_logger().error(f"Depth 변환 오류: {e}")

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.check_and_print()
        except Exception as e:
            self.get_logger().error(f"RGB 변환 오류: {e}")

    def check_and_print(self):
        if self.depth_image is not None and self.rgb_image is not None:
            h, w = self.depth_image.shape[:2]

            # 중앙 좌표 계산 (한 번만)
            if self.x is None or self.y is None:
                self.x = w // 2
                self.y = h // 2
                self.get_logger().info(f"이미지 중앙 좌표 자동 설정: ({self.x}, {self.y})")

            if 0 <= self.x < w and 0 <= self.y < h:
                now = self.get_clock().now()
                time_diff = (now - self.last_logged_time).nanoseconds / 1e9

                if time_diff > 0.5:
                    depth_value = self.depth_image[self.y, self.x]
                    rgb_value = self.rgb_image[self.y, self.x]  # BGR

                    # 깊이 단위 환산
                    if self.depth_image.dtype == np.uint16:
                        depth_in_m = depth_value / 1000.0
                    else:
                        depth_in_m = depth_value

                    self.get_logger().info(
                        f"[{now.to_msg().sec}.{now.to_msg().nanosec}] "
                        f"중앙 좌표 ({self.x},{self.y}) → 깊이: {depth_in_m:.3f} m | RGB (BGR): {rgb_value}"
                    )
                    self.last_logged_time = now
            else:
                self.get_logger().warn("중앙 좌표가 이미지 범위를 벗어났습니다.")

def main():
    rclpy.init()
    node = DepthRGBQueryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()