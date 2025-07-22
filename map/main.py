import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions
from .waypoint_manager import WaypointManager
from .navigator import Navigator
from .battery_monitor import BatteryMonitor
from .docking_manager import DockingManager
from .delivery_queue import DeliveryQueue
import time

class DeliveryManager(Node):
    def __init__(self):
        super().__init__('delivery_manager')

        self.waypoint_manager = WaypointManager()
        self.navigator = Navigator(self)
        self.battery_monitor = BatteryMonitor(namespace='/robot8/')
        self.docking_manager = DockingManager(self.navigator)
        self.delivery_queue = DeliveryQueue(self)

        self.max_retry = 3
        self.is_docked = True

        self.get_logger().info("🚚 배송 매니저 시작됨")

    def main_loop(self):
        # 1. 배터리 부족 → 도킹 및 충전
        if self.battery_monitor.is_battery_low():
            self.get_logger().info("🔋 배터리 부족 → 도킹 시작")
            self.go_to_waypoint_by_name("gohome")
            self.docking_manager.dock()
            self.is_docked = True

            while not self.battery_monitor.is_battery_enough_to_depart():
                self.get_logger().info("⚡ 충전 중... (5초 후 재확인)")
                time.sleep(5)

            self.get_logger().info("🔋 충전 완료 → 언도킹 및 stay 이동")
            self.docking_manager.undock()
            self.is_docked = False
            self.go_to_waypoint_by_name("stay")  # ✅ stay 여기서만 실행
            return

        # 2. 도킹 상태면 언도킹 + stay 이동
        if self.is_docked:
            self.get_logger().info("🚪 언도킹 후 stay 이동")
            self.docking_manager.undock()
            self.is_docked = False
            self.go_to_waypoint_by_name("stay")  # ✅ 중복 방지

        # 3. 배송 큐 없는 경우 → 대기
        if self.delivery_queue.is_empty():
            self.get_logger().info("📭 배송 큐 없음 → 대기")
            return

        # 4. 배송 큐 처리
        task = self.delivery_queue.get_next_request()
        if len(task) != 2:
            self.get_logger().warn("🚫 유효하지 않은 배송 명령 구조 (hub, home 필수)")
            return

        hub, home = task
        route = [hub, "cross", home, "cross2"]
        raw_waypoints = self.waypoint_manager.get_waypoints_by_names(route)

        goal_poses = []
        for wp in raw_waypoints:
            name, x, y, direction = wp
            pose = self.navigator.navigator.getPoseStamped(
                [x, y],
                TurtleBot4Directions[direction.upper()]
            )
            goal_poses.append(pose)

        if len(goal_poses) != len(route):
            self.get_logger().warn("🚫 일부 waypoint를 찾을 수 없어 작업 생략")
            return

        for i, pose in enumerate(goal_poses):
            name = route[i]
            self.get_logger().info(f"➡ 이동: {name}")
            if not self.try_navigation(pose, name):
                self.get_logger().warn(f"❌ {name} 도달 실패 → stay 이동 및 도킹")
                self.go_to_waypoint_by_name("stay")
                self.docking_manager.dock()
                self.is_docked = True
                return

        self.get_logger().info("📦 배송 완료") 
        self.go_to_waypoint_by_name("stay")

    def try_navigation(self, pose: PoseStamped, name: str) -> bool:
        for attempt in range(1, self.max_retry + 1):
            self.navigator.go_to_pose(pose)
            self.get_logger().info(f"🕒 목표 도달 대기 중... (시도 {attempt})")

            while not self.navigator.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.5)

            result = self.navigator.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if name[:5] not in ("cross", "stay", "gohom"):
                    time.sleep(5)
                self.get_logger().info("✅ 도달 성공")
                return True
            else:
                self.get_logger().warn("⏳ 도달 실패, 재시도")
        return False

    def go_to_waypoint_by_name(self, name: str):
        waypoints = self.waypoint_manager.get_waypoints_by_names([name])
        if not waypoints:
            self.get_logger().warn(f"❌ {name} waypoint 없음")
            return

        pose = self.navigator.navigator.getPoseStamped(
            [waypoints[0][1], waypoints[0][2]],
            TurtleBot4Directions[waypoints[0][3].upper()]
        )
        self.try_navigation(pose, name)

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManager()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            node.main_loop()
            time.sleep(1.0)
    except Exception as e:
        node.get_logger().error(f'❌ 예외 발생: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
