import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from ultralytics import YOLO
import numpy as np
import cv2
import math
import threading
import time


class YoloWaypointNavigator(Node):
    def __init__(self):
        super().__init__('yolo_waypoint_navigator')
        self.bridge = CvBridge()
        self.K = None
        self.lock = threading.Lock()

        ns = self.get_namespace()
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.cmd_vel_pub = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)
        self.position_pub = self.create_publisher(PointStamped, '/lead_car/position_3d', 10)

        self.rgb_image = None
        self.depth_image = None
        self.camera_frame = None
        self.display_image = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.model = YOLO("/home/rokey/rokey4_C1_ws/src/carbot/carbot/my_best.pt")

        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        init_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(init_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.goals = [
            self.navigator.getPoseStamped([-2.3, 0.08], TurtleBot4Directions.EAST),
            self.navigator.getPoseStamped([-2.3, -1.8], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([0.0, -1.9], TurtleBot4Directions.NORTH),
        ]
        self.current_index = 0
        self.obstacle_detected = False
        self.is_cancelled = False

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

        self.timer = self.create_timer(0.5, self.detect_and_control)
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        # Start navigation in separate thread
        threading.Thread(target=self.navigate_waypoints, daemon=True).start()

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id
        self.get_logger().info_once("Camera intrinsics received")

    def rgb_callback(self, msg):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            with self.lock:
                self.rgb_image = image
        except Exception as e:
            self.get_logger().error(f"RGB decode failed: {e}")

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.lock:
                self.depth_image = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def detect_and_control(self):
        with self.lock:
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None

        if rgb is None or depth is None or self.K is None:
            return

        result = self.model(rgb, verbose=False)[0]
        found_car = False

        for det in result.boxes:
            label = self.model.names[int(det.cls[0])]
            if label.lower() != 'car':
                continue

            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())
            u = (x1 + x2) // 2
            v = (y1 + y2) // 2
            z = float(depth[v, u]) / 1000.0

            if not (0.3 < z < 5.0):
                continue

            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]
            X = (u - cx) * z / fx
            Y = (v - cy) * z / fy
            Z = z

            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = self.camera_frame
            pt.point.x = X
            pt.point.y = Y
            pt.point.z = Z
            self.position_pub.publish(pt)

            self.get_logger().info(f"🚘 차량 감지: z={z:.2f}m → 좌표=({X:.2f}, {Y:.2f})")
            found_car = True
            break

        if found_car and not self.obstacle_detected:
            self.get_logger().warn("🟥 차량 감지됨 → Goal 취소")
            self.navigator.cancelNav()
            self.is_cancelled = True
            self.obstacle_detected = True

        elif not found_car and self.obstacle_detected:
            self.get_logger().info("🟩 차량 사라짐 → Goal 재개")
            self.resume_navigation()
            self.obstacle_detected = False

    def navigate_waypoints(self):
        while self.current_index < len(self.goals):
            if not self.obstacle_detected and not self.is_cancelled:
                goal = self.goals[self.current_index]
                self.get_logger().info(f"📍 Waypoint #{self.current_index+1} 이동 중")
                self.navigator.goToPose(goal)
                self.get_logger().info(f"✅ Waypoint #{self.current_index+1} 도달")
                self.current_index += 1
            time.sleep(0.5)

    def resume_navigation(self):
        if self.current_index < len(self.goals):
            threading.Thread(target=self.navigate_waypoints, daemon=True).start()
            self.is_cancelled = False

    def gui_loop(self):
        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.lock:
                img = self.rgb_image.copy() if self.rgb_image is not None else None
            if img is not None:
                cv2.imshow("YOLO Detection", img)
                if cv2.waitKey(1) == 27:
                    self.get_logger().info("Shutdown by ESC")
                    rclpy.shutdown()
                    break


def main():
    rclpy.init()
    node = YoloWaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
