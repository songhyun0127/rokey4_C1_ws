import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import time

class FakeRCPositionPublisher(Node):
    def __init__(self):
        super().__init__('fake_rc_position_publisher')

        self.publisher = self.create_publisher(PointStamped, '/lead_car/position_3d', 10)
        self.timer_period = 0.2  # 5Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.1  # m/s 초기 속도

        self.start_time = time.time()
        self.last_time = self.start_time

        self.get_logger().info("🟢 가짜 RC카 위치 퍼블리셔 시작됨 (/lead_car/position_3d)")

    def timer_callback(self):
        now = time.time()
        dt = now - self.last_time
        elapsed = now - self.start_time
        self.last_time = now

        # ✅ 10초 후 RC카 멈추게 하기
        if elapsed > 5.0:
            self.vx = 0.0  # 정지 상태 시뮬레이션

        # 속도 기반으로 위치 업데이트
        self.x += self.vx * dt

        # 위치 메시지 퍼블리시
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = self.z

        self.publisher.publish(msg)
        self.get_logger().info(f"📍 퍼블리시: x = {self.x:.2f} m, vx = {self.vx:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = FakeRCPositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
