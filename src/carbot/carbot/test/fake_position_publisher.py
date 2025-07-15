import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import time
import math  # ⬅️ y 변화량 추가를 위한 모듈

class FakeRCPositionPublisher(Node):
    def __init__(self):
        super().__init__('fake_rc_position_publisher')

        self.publisher = self.create_publisher(PointStamped, '/lead_car/position_3d', 10)
        self.timer_period = 0.2  # 5Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # -----------------------------------------
        # 🟡 위치 및 속도 초기값 설정
        # -----------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.2  # 초기 전진 속도 [m/s]

        # 🟡 시간 기록 변수
        self.start_time = time.time()
        self.last_time = self.start_time

        self.get_logger().info("🟢 가짜 RC카 위치 퍼블리셔 시작됨 (/lead_car/position_3d)")

    def timer_callback(self):
        now = time.time()
        dt = now - self.last_time
        elapsed = now - self.start_time
        self.last_time = now

        # -----------------------------------------
        # 🟡 [변경 1] 경과 시간에 따라 속도 변경
        #   - 0~5초: 전진 (+0.1 m/s)
        #   - 5~10초: 정지 (0.0 m/s)
        #   - 10초 이후: 후진 (-0.1 m/s)
        # -----------------------------------------
        if elapsed > 10.0:
            self.vx = -0.1  # 후진 시작
        elif elapsed > 5.0:
            self.vx = 0.0   # 정지 유지
        else:
            self.vx = 0.1   # 전진 유지

        # -----------------------------------------
        # 🟡 [변경 2] 위치 업데이트 (x + y 진동 추가)
        # -----------------------------------------
        self.x += self.vx * dt
        self.y = 0.1 * math.sin(elapsed)  # y좌표: 시간에 따른 진동으로 거리 변화 유도

        # 메시지 생성 및 퍼블리시
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = self.z

        self.publisher.publish(msg)

        # 로그 출력
        self.get_logger().info(f"📍 퍼블리시: x = {self.x:.2f} m, y = {self.y:.2f} m, vx = {self.vx:.2f} m/s")

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
