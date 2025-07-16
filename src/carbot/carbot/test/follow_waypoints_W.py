import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class SimpleWaypointFollower(Node):
    def __init__(self):
        super().__init__('simple_waypoint_follower')
        self.navigator = TurtleBot4Navigator()
        
        # Nav2 활성화까지 대기
        self.navigator.waitUntilNav2Active()
        
        # 도킹 상태라면 undock
        if self.navigator.getDockedStatus():
            self.get_logger().info('📍 도킹 상태 → 언도킹 시도')
            self.navigator.undock()
        
        # 목표 좌표 설정
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = -3.0
        goal.pose.position.y = 0.326
        goal.pose.position.z = 0.0
        goal.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.get_logger().info(f"🚗 목표 좌표로 이동 중: ({goal.pose.position.x}, {goal.pose.position.y})")
        self.navigator.goToPose(goal)
        
        # 도착 대기
        while not self.navigator.isTaskComplete():
            pass
        
        # 결과 확인
        result = self.navigator.getResult()
        if result == self.navigator.TaskResult.SUCCEEDED:
            self.get_logger().info("✅ 목표 지점 도착 완료!")
        else:
            self.get_logger().info("❌ 목표 이동 실패")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()