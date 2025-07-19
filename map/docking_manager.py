import time
from nav2_simple_commander.robot_navigator import TaskResult
import rclpy

class DockingManager:
    def __init__(self, navigator):
        self.navigator = navigator

    def dock(self):
        self.navigator.navigator.dock()

        # ✅ 도킹 완료 대기
        self.navigator.node.get_logger().info("🧲 도킹 중...")
        while not self.navigator.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator.node, timeout_sec=0.5)

        result = self.navigator.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.navigator.node.get_logger().info("✅ 도킹 완료")
        else:
            self.navigator.node.get_logger().warn("❌ 도킹 실패")


    def undock(self):
        self.navigator.navigator.undock()
        self.navigator.node.get_logger().info("언도킹 완료")

    def dock_until_charged(self, threshold: float = 0.8):
        self.dock()
        self.navigator.node.get_logger().info("🔌 충전 중... 80% 이상 충전되면 출발 가능")

        battery_monitor = self.navigator.node.battery_monitor
        start_time = time.time()
        timeout = 60  # 최대 5분 대기

        while battery_monitor.last_battery_percent < threshold:
            if time.time() - start_time > timeout:
                self.navigator.node.get_logger().warn("❗ 충전 타임아웃. 강제 진행")
                break
            self.navigator.node.get_logger().info(f"⚡ 현재 배터리: {battery_monitor.last_battery_percent:.1f}%")
            rclpy.spin_once(self.navigator.node, timeout_sec=1.0)
            time.sleep(1)

        self.navigator.node.get_logger().info("🔋 충전 완료. 배송 재개 가능.")