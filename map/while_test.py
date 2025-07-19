import rclpy
from rclpy.node import Node
from .delivery_queue import DeliveryQueue
from .waypoint_manager import WaypointManager
from .navigator import Navigator
from .battery_monitor import BatteryMonitor
from .docking_manager import DockingManager
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions
from nav2_simple_commander.robot_navigator import TaskResult
import time


class DeliveryExecutor(Node):
    def __init__(self):
        super().__init__('delivery_executor')

        self.waypoint_manager = WaypointManager()
        self.navigator = Navigator(self)
        self.delivery_queue = DeliveryQueue(self)
        self.battery_monitor = BatteryMonitor(namespace=self.get_namespace())
        self.docking_manager = DockingManager(self.navigator)

        self.get_logger().info("✅ 배송 시스템 초기화 완료")

    def control_loop(self):
        self.get_logger().info("🔄 control_loop 재실행됨")

        if self.delivery_queue.is_empty():
            self.get_logger().info("🕓 대기 중 (큐 비어있음)")
            return

        target_names = self.delivery_queue.get_next_request()
        self.get_logger().info(f"📦 배송 요청 수신: {target_names}")

        waypoint_data = self.waypoint_manager.get_waypoints_by_names(target_names)

        goal_poses = []
        for name, x, y, direction in waypoint_data:
            pose = self.navigator.navigator.getPoseStamped(
                [x, y],
                TurtleBot4Directions[direction.upper()]
            )
            goal_poses.append(pose)
            self.get_logger().info(f"➡️ {name} → x={x}, y={y}, 방향={direction}")

        if not goal_poses:
            self.get_logger().warn("❌ 유효한 waypoint 없음, 이동 생략")
            return

        # ✅ 1. 배터리 부족 시 도킹 & 충전 대기
        if self.battery_monitor.is_battery_low():
            self.get_logger().info("🔋 배터리 부족, 도킹 시작")
            self.docking_manager.dock_until_charged(threshold=80.0)

        # ✅ 2. 언도킹
        self.get_logger().info("🚪 언도킹 시작")
        self.docking_manager.undock()

        # ✅ 3. 배송지로 이동
        self.navigator.follow_waypoints(goal_poses)

        # ✅ 복귀 시작
        return_pose = self.navigator.navigator.getPoseStamped(
            [0.4, 1.5],
            TurtleBot4Directions.EAST
        )
        self.get_logger().info("🔁 복귀 시작 (초기 위치)")
        self.navigator.go_to_pose(return_pose)

        # ✅ 복귀 완료 대기
        self.get_logger().info("⌛ 복귀 완료 대기 중...")
        while not self.navigator.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.5)

        # ✅ 복귀 결과 확인
        result = self.navigator.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("✅ 복귀 완료")
            self.docking_manager.dock()
            self.get_logger().info("✅ 도킹 로직 종료됨, 다음 주문 대기 진입")
        else:
            self.get_logger().warn("❌ 복귀 실패. 도킹 생략")


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryExecutor()

    try:
        while rclpy.ok():
            node.control_loop()
            rclpy.spin_once(node, timeout_sec=0.5)
            time.sleep(1.0)  # 제어 주기 1초
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
