import rclpy
import numpy as np
import time
import random
from collections import deque
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, DurabilityPolicy # [추가] QoS 설정용
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Float64
from f110_msgs.msg import WpntArray

class TrainingSupervisor(Node):
    def __init__(self):
        super().__init__('training_supervisor')

        # --- [설정 파라미터] ---
        self.min_collision_dist = 0.01  # (m) 이보다 가까우면 충돌로 간주
        self.oscillation_threshold = 6  # (회) 1초에 조향 부호가 8번 이상 바뀌면 오실레이션
        self.stuck_speed_threshold = 0.1 # (m/s) 이보다 느리면 정체
        self.stuck_time_limit = 2.0      # (초) 정체 허용 시간
        self.success_duration = 60.0     # (초) 이 시간 동안 생존하면 성공으로 간주

        # --- [상태 변수] ---
        self.start_time = self.get_clock().now()
        self.last_stuck_time = self.get_clock().now()
        self.steering_history = deque(maxlen=40) # 1초치 조향 데이터 (40Hz)
        self.global_waypoints = None 

        
        # --- [통신 설정] ---
        # 감시 대상
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(WpntArray, '/global_waypoints', self.waypoints_cb, 10)
        self.create_subscription(Odometry, 'early_fusion/odom', self.odom_cb, 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self.drive_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        # 제어 명령
        # 1. 판정 결과 (Controller에게 보냄)
        self.status_pub = self.create_publisher(String, '/training/episode_status', 10)
        # 2. 차량 리셋 (Sim에게 보냄)
        self.reset_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 3. 마찰계수 변경 (Sim에게 보냄 - 커스텀 기능 필요)
        #self.friction_pub = self.create_publisher(Float64, '/gym/update_friction', 10)

        # 주기적 체크 (성공 판정용)
        self.create_timer(1.0, self.check_success)
        
        self.get_logger().info("Training Supervisor Started:")

    # --- [감지 로직] ---

    def scan_cb(self, msg):
        # 1. 충돌 감지
        ranges = np.array(msg.ranges)
        # inf/nan 제거
        ranges = ranges[np.isfinite(ranges)]
        
        if len(ranges) > 0:
            min_dist = np.min(ranges)
            if min_dist < self.min_collision_dist:
                self.reset_episode(reason="CRASH")

    def drive_cb(self, msg):
        # 2. 오실레이션 감지
        steer = msg.drive.steering_angle
        self.steering_history.append(steer)
        
        if len(self.steering_history) >= 20:
            # Zero Crossing Rate 계산
            data = np.array(self.steering_history)
            # 부호가 바뀐 횟수
            crossings = np.where(np.diff(np.sign(data)))[0]
            
            if len(crossings) > self.oscillation_threshold:
                self.reset_episode(reason="OSCILLATION")

    def odom_cb(self, msg):
        # 3. 정체(Stuck) 감지
        speed = msg.twist.twist.linear.x
        if abs(speed) > self.stuck_speed_threshold:
            self.last_stuck_time = self.get_clock().now()
        else:
            # 속도가 낮은 상태가 지속되는지 확인
            elapsed = (self.get_clock().now() - self.last_stuck_time).nanoseconds / 1e9
            if elapsed > self.stuck_time_limit:
                # 시작 직후(5초 이내)에는 봐줌 (출발 준비 등)
                episode_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                if episode_time > 5.0:
                    self.reset_episode(reason="STUCK")

    def map_cb(self, msg):
        pass

    def waypoints_cb(self, msg):
        if self.global_waypoints is None:
            self.get_logger().info(f"Global Waypoints Received: {len(msg.wpnts)} points")
        self.global_waypoints = msg.wpnts

    def check_success(self):
        # 4. 성공 판정 (시간 경과)
        episode_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if episode_time > self.success_duration:
            self.reset_episode(reason="TIMEOUT_SUCCESS")

    # --- [리셋 및 판정 로직] ---

    def get_random_free_pose(self):
        if self.global_waypoints is None or len(self.global_waypoints) == 0: 
            self.get_logger().warn("Waiting for Global Waypoints...")
            return None

        # 1. 랜덤 웨이포인트 하나 선택
        idx = random.randint(0, len(self.global_waypoints) - 1)
        wp = self.global_waypoints[idx]
        
        # 2. 기본 좌표 (트랙 중앙)
        wx = wp.x_m
        wy = wp.y_m
        wyaw = wp.psi_rad # 웨이포인트가 가리키는 방향 (트랙 진행 방향)

        # 3. 노이즈 추가 (다양성 확보)
        # 너무 중앙에서만 시작하지 않도록 약간의 위치/각도 노이즈를 섞음
        # (트랙 폭을 고려해 ±0.3m 정도는 안전함)
        noise_x = random.uniform(-0.2, 0.2)
        noise_y = random.uniform(-0.2, 0.2)
        noise_yaw = random.uniform(-0.1, 0.1) # ±5도 정도 틀어서 시작

        return wx + noise_x, wy + noise_y, wyaw + noise_yaw

    def reset_episode(self, reason):
        # 쿨다운 (너무 잦은 리셋 방지, 1초)
        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds / 1e9 < 1.0:
            return

        # A. 판정 메시지 전송 (Controller에게)
        status_msg = String()
        if reason in ["CRASH", "OSCILLATION", "STUCK"]:
            status_msg.data = "FAIL"
            self.get_logger().warn(f"Episode FAILED ({reason}) -> Discard Data")
        else:
            status_msg.data = "SUCCESS"
            self.get_logger().info(f"Episode SUCCESS ({reason}) -> Commit Data")
        
        self.status_pub.publish(status_msg)
        
        # (Controller가 메시지 받을 시간 확보)
        time.sleep(0.1)

        # B. 차량 리셋 (Initial Pose)
        tx, ty, tyaw = self.get_random_free_pose()
        
        reset_msg = PoseWithCovarianceStamped()
        reset_msg.header.stamp = self.get_clock().now().to_msg()
        reset_msg.header.frame_id = "map"
        reset_msg.pose.pose.position.x = tx
        reset_msg.pose.pose.position.y = ty
        
        q = Rotation.from_euler('z', tyaw).as_quat()
        reset_msg.pose.pose.orientation.z = q[2]
        reset_msg.pose.pose.orientation.w = q[3]
        
        self.reset_pub.publish(reset_msg)

        # C. 마찰계수 랜덤화 (0.4 ~ 1.2) <= TODO
        # (시뮬레이터가 이 토픽을 구독하여 물리 엔진을 업데이트해야 함)
        #new_mu = random.uniform(0.4, 1.2)
        #self.friction_pub.publish(Float64(data=new_mu))
        #self.get_logger().info(f"Reset Done. New Friction: {new_mu:.2f}")

        # 타이머 리셋
        self.start_time = self.get_clock().now()
        self.last_stuck_time = self.get_clock().now()
        self.steering_history.clear()

def main():
    rclpy.init()
    node = TrainingSupervisor()
    rclpy.spin(node)
    rclpy.shutdown()