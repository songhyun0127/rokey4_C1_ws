import rclpy
from rclpy.node import Node
from delivery_queue import DeliveryQueue
from waypoint_manager import WaypointManager

class DeliveryTestLogger(Node):
    def __init__(self):
        super().__init__('delivery_test_logger')

        # WaypointManager 초기화 (JSON 로딩 포함)
        self.waypoint_manager = WaypointManager()
        self.get_logger().info("✅ WaypointManager 로드 완료")

        # DeliveryQueue 초기화 및 구독 시작
        self.delivery_queue = DeliveryQueue(self)

        # 주기적으로 큐에서 꺼내어 좌표 확인하는 타이머 (2초 간격)
        self.timer = self.create_timer(2.0, self.check_queue)

    def check_queue(self):
        if self.delivery_queue.is_empty():
            self.get_logger().info("⏳ 대기 중... 수신된 Waypoint 없음")
            return

        target_names = self.delivery_queue.get_next_request()
        self.get_logger().info(f"📦 받은 요청: {target_names}")

        waypoints = self.waypoint_manager.get_waypoints_by_names(target_names)
        for name, x, y, direction in waypoints:
            self.get_logger().info(f"🧭 {name} → x: {x}, y: {y}, direction: {direction}")

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryTestLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
