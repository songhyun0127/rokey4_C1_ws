import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from ultralytics import YOLO
import numpy as np
import cv2
import math
import threading


class YoloFollower(Node):
    def __init__(self):
        super().__init__('yolo_follower_node')

        self.bridge = CvBridge()
        self.K = None
        self.lock = threading.Lock()

        ns = self.get_namespace()
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.cmd_vel_pub = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)

        self.rgb_image = None
        self.depth_image = None
        self.camera_frame = None
        self.display_image = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.model = YOLO("/home/hihigiig101/Downloads/my_best.pt")

        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        init_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(init_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

        self.start_timer = self.create_timer(5.0, self.start_detection)
        self.timer = None

        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id
        self.get_logger().info(f"Camera intrinsics: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}")

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

    def start_detection(self):
        self.get_logger().info("Starting YOLO-based detection")
        self.timer = self.create_timer(0.5, self.detect_and_navigate)
        self.start_timer.cancel()

    def detect_and_navigate(self):
        with self.lock:
            rgb = self.rgb_image.copy() if self.rgb_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None

        if rgb is None or depth is None or self.K is None:
            return

        result = self.model(rgb, verbose=False)[0]
        for det in result.boxes:
            cls = int(det.cls[0])
            label = self.model.names[cls]
            conf = float(det.conf[0])
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

            if label.lower() != 'car':
                continue
            
            # 박스 그리기
            cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f"{label} {conf:.2f}"
            cv2.putText(rgb, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            u = (x1 + x2) // 2
            v = (y1 + y2) // 2
            z = float(depth[v, u]) / 1000.0  # Convert mm to meters if necessary

            if not (0.3 < z < 5.0):
                self.get_logger().warn("Invalid depth value")
                continue

            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]
            X = (u - cx) * z / fx
            Y = (v - cy) * z / fy
            Z = z

            # 거리 기반 속도 제어 파라미터
            target_distance = 1.0  # 목표 거리 (m)
            max_speed = 0.3        # 최대 속도 (m/s)
            min_speed = 0.15

            # 너무 가까우면 정지, 아니면 비례제어 속도 계산
            if z < 0.4:
                speed = 0.0
            else:
                speed = max(min_speed, min(max_speed, (z - target_distance) * 0.5))

            # Twist 메시지 생성 및 속도 발행 
            twist = Twist()
            twist.linear.x = speed

            # 속도 버블리시
            self.cmd_vel_pub.publish(twist)
            
            # 화면 출력
            self.get_logger().info(f"[car] z={z:.2f}m → speed={speed:.2f} m/s")

            pt_camera = PointStamped()
            pt_camera.header.stamp = Time().to_msg()
            pt_camera.header.frame_id = self.camera_frame
            pt_camera.point.x, pt_camera.point.y, pt_camera.point.z = X, Y, Z

            # try:
            #     pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=Duration(seconds=1.0))
            #     goal = PoseStamped()
            #     goal.header.frame_id = 'map'
            #     goal.header.stamp = self.get_clock().now().to_msg()
            #     goal.pose.position.x = pt_map.point.x
            #     goal.pose.position.y = pt_map.point.y
            #     goal.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            #     self.get_logger().info(f"Navigating to: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")
            #     self.navigator.goToPose(goal)

            #     break  # Navigate to first person only
            # except Exception as e:
            #     self.get_logger().warn(f"TF failed: {e}")

        self.display_image = rgb

    def gui_loop(self):
        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.lock:
                img = self.display_image.copy() if self.display_image is not None else None
            if img is not None:
                cv2.imshow("YOLO Detection", img)
                if cv2.waitKey(1) == 27:
                    self.get_logger().info("Shutdown by ESC")
                    rclpy.shutdown()
                    break


def main():
    rclpy.init()
    node = YoloFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()