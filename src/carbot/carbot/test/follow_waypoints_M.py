import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.navigator = TurtleBot4Navigator()

        # 도킹 상태이면 undock 전까지 pose 설정 불가
        if not self.navigator.getDockedStatus():
            self.get_logger().info('📍 도킹 상태 아님 → 도킹 시도')
            self.navigator.dock()

        # ✅ 초기 위치 수동 설정
        self.get_logger().info("📌 초기 위치 설정 중...")
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.position.x = -0.26513
        init_pose.pose.position.y = -0.10379
        init_pose.pose.position.z = 0.0
        init_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.9987, w=0.05099)

        self.navigator.setInitialPose(init_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        # ✅ 이동할 Waypoint 설정
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = -3.0
        goal.pose.position.y = 0.326
        goal.pose.position.z = 0.0
        goal.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info(f"🚗 Waypoint로 이동 중: ({goal.pose.position.x}, {goal.pose.position.y})")
        self.navigator.goToPose(goal)

        # 도착 대기
        self.navigator.waitUntilNav2GoalReached()

        self.get_logger().info("✅ 목표 지점 도착 완료!")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
