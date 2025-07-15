import time
import math
import os
import sys
import rclpy
import threading
from queue import Queue
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
import cv2
import numpy as np

class YOLOImageSubscriber(Node):
    def __init__(self, model):
        super().__init__('yolo_image_subscriber')
        self.model = model
        self.bridge = CvBridge()
        self.image_queue = Queue(maxsize=1)
        self.should_shutdown = False
        self.classNames = model.names if hasattr(model, 'names') else ['Object']

        self.subscription = self.create_subscription(
            Image,
            '/robot8/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10)

        self.thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.thread.start()
        self.latest_depth_image = None
        self.latest_rgb_image = None

        self.depth_sub = self.create_subscription(
            Image,
            '/robot8/oakd/depth/image_raw',  # 실제 토픽명에 맞게 수정
            self.depth_callback,
            10)

        self.rgb_sub = self.create_subscription(
            Image,
            '/robot8/oakd/rgb/image_raw',  # 실제 토픽명에 맞게 수정
            self.rgb_callback,
            10)
    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB image conversion failed: {e}")
    def get_distance(self, x, y):
        if self.latest_depth_image is None:
            return None

        h, w = self.latest_depth_image.shape[:2]
        x = min(max(0, x), w - 1)
        y = min(max(0, y), h - 1)

        depth_value = self.latest_depth_image[y, x]

        if isinstance(depth_value, (np.uint16, int)) and depth_value > 0:
            return depth_value / 1000.0  # mm → meter
        elif isinstance(depth_value, float):
            return depth_value
        else:
            return None


    def listener_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not self.image_queue.full():
                self.image_queue.put(img)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def detection_loop(self):
        while not self.should_shutdown:
            try:
                img = self.image_queue.get(timeout=0.5)
            except:
                continue

            results = self.model.predict(img, stream=True)

            for r in results:
                if not hasattr(r, 'boxes') or r.boxes is None:
                    continue
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0]) if box.cls is not None else 0
                    conf = float(box.conf[0]) if box.conf is not None else 0.0
                    # ✅ [추가 7] 바운딩 박스 중심 좌표
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # ✅ [추가 8] 중심에서 거리 추정
                    distance = self.get_distance(cx, cy)

                    # ✅ [추가 9] 중심에서 RGB(BGR) 추출
                    rgb_value = None
                    if self.latest_rgb_image is not None:
                        h, w = self.latest_rgb_image.shape[:2]
                        if 0 <= cx < w and 0 <= cy < h:
                            rgb_value = tuple(self.latest_rgb_image[cy, cx])  # BGR

                    # ✅ [추가 10] 표시할 텍스트 생성
                    label = f"{self.classNames[cls]} {conf:.2f}"
                    if distance is not None:
                        label += f" {distance:.2f}m"
                    if rgb_value is not None:
                        label += f" BGR:{rgb_value}"

                    # label = f"{self.classNames[cls]} {conf:.2f}"
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 2)

            cv2.imshow("YOLOv8 Detection", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Shutdown requested via 'q'")
                self.should_shutdown = True
                break
    



def main():
    model_path = input("Enter path to model file (.pt): ").strip()

    if not os.path.exists(model_path):
        print(f"File not found: {model_path}")
        sys.exit(1)

    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"Unsupported model format: {suffix}")
        sys.exit(1)

    rclpy.init()
    node = YOLOImageSubscriber(model)

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested via Ctrl+C.")
    finally:
        node.should_shutdown = True
        node.thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()
