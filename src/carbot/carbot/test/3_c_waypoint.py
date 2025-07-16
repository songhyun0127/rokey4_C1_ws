import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import Twist
import time
import math
class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.navigator = TurtleBot4Navigator()
        self.human_detected = False
        self._last_msg_time = time.time()
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'human_check', self.human_check_callback, 10)
    def human_check_callback(self, msg):
        self.human_detected = msg.data
        self._last_msg_time = time.time()
    def rotate_in_place(self, angle_degrees, angular_speed=0.5):
        twist = Twist()
        twist.angular.z = math.copysign(angular_speed, angle_degrees)
        duration = abs(math.radians(angle_degrees)) / angular_speed
        self.navigator.info(f'Rotating {angle_degrees} degrees...')
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.navigator.info('Rotation complete.') # ch
    def step_through_waypoints(self, waypoints, home_pose):
        stop_indices = [i for i, wp in enumerate(waypoints) if wp.get('stop', False)]
        last_stop_index = stop_indices[-1] if stop_indices else -1
        for i, wp in enumerate(waypoints):
            pose = wp.get('pose', None)
            rotate = wp.get('rotate', None)
            stop = wp.get('stop', False)
            is_last_stop = (i == last_stop_index)
            if pose:
                self.navigator.info(f'Navigating to waypoint {i+1}/{len(waypoints)}')
                self.navigator.startToPose(pose)
                while not self.navigator.isTaskComplete():
                    rclpy.spin_once(self)
                self.navigator.info(f'Arrived at waypoint {i+1}')
            if rotate:
                self.rotate_in_place(rotate)
            if stop:
                self.human_detected = False
                self.navigator.info("Waiting for human_check to be True...")
                while not self.human_detected and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.2)
                self.navigator.info("human_check received. Proceeding...")
                if is_last_stop:
                    self.navigator.info("Monitoring human presence to return Home after disappearance...")
                    start_time = None
                    while rclpy.ok():
                        rclpy.spin_once(self, timeout_sec=0.5)
                        if not self.human_detected:
                            if start_time is None:
                                start_time = time.time()
                                self.navigator.info("human_check lost. Starting return timer...")
                            elif time.time() - start_time >= 10.0:
                                self.navigator.info("Human disappeared. Returning Home...")
                                self.navigator.startToPose(home_pose)
                                while not self.navigator.isTaskComplete():
                                    rclpy.spin_once(self)
                                self.navigator.info("Docking...")
                                self.navigator.dock()
                                return
                        else:
                            start_time = None
def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    navigator = node.navigator
    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    navigator.undock()
    navigator.info('Welcome to the mail delivery service.')
    # :압정: 좌표 변수 정의
    wp1 = [-2.225916920936506, 0.4538073747945547]
    wp2 = [-2.4833483182920872, -2.1090604231553223]
    wp_goal = [-0.3763219887829313, -2.3521555998360024]
    wp2_1 = [-1.278993, -0.200784]
    wp2_2 = [-1.330508, -0.593572]
    wp2_3 = [-1.874826, -0.604343]
    wp2_goal = [-2.024864, -2.160810]
    home_pose = navigator.getPoseStamped([-0.6, 0.0], TurtleBot4Directions.NORTH)
    goal_options = [
        {'name': 'Position 1',
         'waypoints': [
            {'rotate': -90, 'stop': True},
            {'rotate': 90, 'stop': False},
            {'pose': navigator.getPoseStamped(wp1, TurtleBot4Directions.SOUTH)},
            {'rotate': 180, 'stop': True},
            {'rotate': -90, 'stop': False},
            {'pose': navigator.getPoseStamped(wp2, TurtleBot4Directions.EAST)},
            {'rotate': 180, 'stop': True},
            {'rotate': -90, 'stop': False},
            {'pose': navigator.getPoseStamped(wp_goal, TurtleBot4Directions.NORTH)},
            {'rotate': 180, 'stop': True},
         ]},
        {
        'name': 'Position 2',
        'waypoints': [
            {'rotate': -90, 'stop': True},  # 제자리 시계방향 90도 회전 후 사람 대기
            {'pose': navigator.getPoseStamped(wp2_1, TurtleBot4Directions.EAST)},  # wp2_1로 이동
            {'rotate': 180, 'stop': True},  # 제자리 회전 후 사람 대기
            {'pose': navigator.getPoseStamped(wp2_2, TurtleBot4Directions.SOUTH)},  # wp2_2로 이동
            {'pose': navigator.getPoseStamped(wp2_3, TurtleBot4Directions.EAST)},   # wp2_3로 이동
            {'rotate': 180, 'stop': True},  # 정지하고 인식 대기
            {'rotate': 90, 'stop': False},  # 반시계 90도 회전
            {'pose': navigator.getPoseStamped(wp2_goal, TurtleBot4Directions.EAST)},  # 목표 지점
            {'rotate': 180, 'stop': True},  # 마지막 정지 + human_check 사라지면 복귀
        ]
    },
        {'name': 'Exit', 'waypoints': []},
    ]
    while rclpy.ok():
        options_str = 'Please enter the number corresponding to the desired robot goal position:\n'
        for i, option in enumerate(goal_options):
            options_str += f'    {i}. {option["name"]}\n'
        raw_input = input(f'{options_str}Selection: ')
        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'Invalid selection: {raw_input}')
            continue
        if selected_index < 0 or selected_index >= len(goal_options):
            navigator.error(f'Selection out of range: {selected_index}')
            continue
        selected_goal = goal_options[selected_index]
        if selected_goal['name'] == 'Exit':
            navigator.info('Going Home...')
            if not navigator.getDockedStatus():
                navigator.startToPose(home_pose)
                while not navigator.isTaskComplete():
                    rclpy.spin_once(node)
                navigator.info('Docking before exiting...')
                navigator.dock()
            else:
                navigator.info('Already docked. No action needed.')
            break
        else:
            navigator.info(f'Navigating step‑by‑step to {selected_goal["name"]}')
            node.step_through_waypoints(selected_goal['waypoints'], home_pose)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()