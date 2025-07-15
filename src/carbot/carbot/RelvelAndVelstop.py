import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Bool
from rclpy.time import Time


class RelativeVelocityAndStopDetector(Node):
    def __init__(self):
        super().__init__('relative_velocity_estimator')

        # ---------------------------------------------
        # [1단계] 정지 판단을 위한 임계값을 파라미터로 선언
        # 사용자가 launch 파일이나 CLI로 조정 가능
        # ---------------------------------------------
        self.declare_parameter('velocity_threshold', 0.05)     # [m/s] 정지로 간주할 최대 상대 속도
        self.declare_parameter('stop_time_threshold', 1.0)     # [sec] 정지로 간주할 최소 지속 시간

        # 파라미터에서 실제 값 가져오기
        self.velocity_threshold = self.get_parameter('velocity_threshold').get_parameter_value().double_value
        self.stop_time_threshold = self.get_parameter('stop_time_threshold').get_parameter_value().double_value

        # ---------------------------------------------
        # [2단계] 상태 변수 초기화
        # RC카의 이전 위치 및 시간, 정지 시간 누적용
        # ---------------------------------------------
        self.prev_pos = None
        self.prev_time = None
        self.stop_duration = 0.0
        self.is_stopped = False

        # ---------------------------------------------
        # [3단계] RC카 위치를 수신할 토픽 구독
        # 위치는 map 기준 (YOLO+Depth 노드에서 publish)
        # ---------------------------------------------
        self.create_subscription(
            PointStamped,
            '/lead_car/position_3d',
            self.position_callback,
            10
        )

        # ---------------------------------------------
        # [4단계] 추정된 상대 속도 및 정지 상태 퍼블리시
        # 다른 노드(AMR 등)가 이 값을 보고 제어 판단함
        # ---------------------------------------------
        self.vel_pub = self.create_publisher(Float32, '/lead_car/relative_velocity', 10)
        self.stop_pub = self.create_publisher(Bool, '/lead_car/is_stopped', 10)

        self.get_logger().info("✅ RelvelAndVelstop 노드가 시작되었습니다")

    # -------------------------------------------------
    # [5단계] RC카 위치 수신 시 호출되는 콜백
    # 이전 위치와 비교하여 상대 속도 계산 및 정지 판단
    # -------------------------------------------------
    def position_callback(self, msg: PointStamped):
        self.get_logger().info(f"📥 위치 수신됨: x = {msg.point.x:.2f}, y = {msg.point.y:.2f}")

        current_pos = msg.point
        current_time = Time.from_msg(msg.header.stamp).nanoseconds / 1e9  # 초 단위

        # 초기에는 비교 대상 없음 → 현재 상태 저장 후 리턴
        if self.prev_pos is None or self.prev_time is None:
            self.prev_pos = current_pos
            self.prev_time = current_time
            self.get_logger().info("🕒 첫 위치 수신됨 - 비교 생략")
            return

        # 시간 간격 계산
        dt = current_time - self.prev_time
        if dt <= 0.0:
            self.get_logger().warn("⚠️ 시간 간격이 0 이하입니다. 계산을 건너뜁니다.")
            return

        # 부호 포함 상대 속도 계산
        v_rel = self.compute_signed_relative_velocity(current_pos, self.prev_pos, dt)

        # 정지 상태 판단
        is_stopped = self.check_stopped(abs(v_rel), dt)

        # 퍼블리시 및 로그 출력
        self.publish_outputs(v_rel, is_stopped)

        # 상태 갱신
        self.prev_pos = current_pos
        self.prev_time = current_time

        
    # -------------------------------------------------
    # [6단계] 상대 속도 계산 함수
    # 현재와 이전 위치(x, y 기준) 거리 변화량 / 시간
    # -------------------------------------------------
    def compute_signed_relative_velocity(self, curr, prev, dt):
        # -----------------------------------------------------
        # RC카의 위치(Point)를 기준으로 "AMR과의 거리"를 계산하여,
        # 두 시점 간 거리 변화량을 이용해 부호 있는 상대 속도 추정
        #
        # ✅ 계산 방식:
        #   - d_prev: 이전 시점에서의 RC카까지의 거리
        #   - d_now : 현재 시점에서의 RC카까지의 거리
        #   - Δd = d_now - d_prev (거리 변화량)
        #   - v_rel_signed = Δd / dt (부호 포함 속도)
        #
        # ✅ 부호 의미:
        #   - + : 앞차가 빨라짐 (거리 벌어짐)
        #   - - : 앞차가 느려짐 (거리 좁아짐)
        #   - 0 : 상대 거리 유지 (또는 정지)
        # -----------------------------------------------------
        d_now = (curr.x**2 + curr.y**2)**0.5
        d_prev = (prev.x**2 + prev.y**2)**0.5
        delta_d = d_now - d_prev
        v_rel_signed = delta_d / dt if dt > 0 else 0.0

        # 로그로 방향성 설명 추가
        direction = (
            "앞차가 더 빠름 (거리 증가)"
            if v_rel_signed > 0 else
            "앞차가 느림 (거리 감소)"
            if v_rel_signed < 0 else
            "정지 또는 유지"
        )
        self.get_logger().info(
            f"상대 속도 계산됨: {v_rel_signed:.3f} m/s | {direction}"
        )

        return v_rel_signed


    # -------------------------------------------------
    # [7단계] 정지 상태 판단 함수
    # 일정 속도 이하가 일정 시간 이상 유지되면 정지로 간주
    # -------------------------------------------------
    def check_stopped(self, v_rel, dt):
        self.get_logger().info(f"🧪 정지 판단중 | 속도: {v_rel:.3f} m/s | 누적 정지 시간: {self.stop_duration:.2f} s")
        if v_rel < self.velocity_threshold:
            self.stop_duration += dt
            if self.stop_duration >= self.stop_time_threshold:
                if not self.is_stopped:
                    self.get_logger().info("🛑 앞차 정지 감지됨")
                self.is_stopped = True
        else:
            if self.is_stopped:
                self.get_logger().info("🟢 앞차 다시 이동 시작")
            self.stop_duration = 0.0
            self.is_stopped = False

        return self.is_stopped

    # -------------------------------------------------
    # [8단계] 속도 및 정지 상태 퍼블리시 함수
    # 다른 노드들이 참고할 수 있도록 결과 토픽 전송
    # -------------------------------------------------
    def publish_outputs(self, v_rel, is_stopped):
        self.vel_pub.publish(Float32(data=v_rel))
        self.stop_pub.publish(Bool(data=is_stopped))
        self.get_logger().info(f"[📏 속도] {v_rel:.3f} m/s | [정지 상태] {is_stopped}")

# -----------------------------------------------------
# [9단계] 노드 실행 엔트리포인트
# ros2 run or launch 파일에서 실행됨
# -----------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RelativeVelocityAndStopDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("❎ 사용자 종료 (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
