import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class FollowWaypointsNode(Node):
    def __init__(self):
        super().__init__('follow_waypoints_node')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_cb, 10)

        # Waypoint list (map 기준 좌표)
        self.waypoints = [
            self.make_pose(2.0, 0.0),
            self.make_pose(4.0, 1.0),
            self.make_pose(6.0, 2.0),
            self.make_pose(8.0, 2.0),
        ]
        self.index = 0
        self.obstacle_detected = False
        self.goal_sent = False
        self.goal_cancelled = False

        self.get_logger().info("🧭 FollowWaypointsNode started")

        # 시작 조건: Nav2 준비되면 첫 goal 전송
        self.timer = self.create_timer(1.0, self.update)

    def obstacle_cb(self, msg: Bool):
        self.obstacle_detected = msg.data
        if self.obstacle_detected and self.goal_sent and not self.goal_cancelled:
            self.cancel_goal()

    def update(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info("🏁 모든 waypoint 도달 완료")
            self.timer.cancel()
            return

        if not self.goal_sent and not self.obstacle_detected:
            self.send_goal(self.waypoints[self.index])

    def send_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ Nav2 서버 미사용 가능")
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_cb)\
            .add_done_callback(self.goal_done_cb)

        self.goal_sent = True
        self.goal_cancelled = False
        self.get_logger().info(f"🚀 Goal #{self.index+1} 전송")

    def cancel_goal(self):
        if self.nav_client.wait_for_server(timeout_sec=2.0):
            self.nav_client.cancel_all_goals()
            self.goal_cancelled = True
            self.goal_sent = False
            self.get_logger().warn("🛑 장애물 감지 → goal 취소")

    def goal_done_cb(self, future):
        result = future.result().result
        if result is not None and result.error_code == 0:
            self.get_logger().info(f"✅ Goal #{self.index+1} 완료")
            self.index += 1
            self.goal_sent = False
        else:
            self.get_logger().warn(f"⚠️ Goal 실패 or 취소됨")

    def feedback_cb(self, feedback):
        # 필요 시 거리 출력 가능
        pass

    def make_pose(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = FollowWaypointsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
