import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
import time

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        self.create_subscription(Float32, '/angle', self.angle_callback, 10)
        self.create_subscription(Float32, '/deg', self.deg_callback, 10)
        self.create_subscription(Bool, '/deg_valid', self.deg_valid_callback, 10)

        # lane change 명령 받기
        self.create_subscription(String, '/lane_change_cmd', self.lane_change_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.angle_data = 0.0
        self.deg_data = 0.0
        self.deg_valid = False

        # 강제조향 변수
        self.force_turn = False
        self.force_turn_end_time = 0.0
        self.force_turn_z = 0.0

        self.linear_speed = 0.10
        self.k_angle = 0.002
        self.k_deg = 0.024

        self.z_filtered = 0.0

        self.timer = self.create_timer(0.01, self.control_callback)

    # 차선 변경 명령
    def lane_change_callback(self, msg):
        key = msg.data.lower().strip()

        if key == 'a':   # 왼쪽 차선 변경 → 왼쪽으로 꺾기
            self.force_turn = True
            self.force_turn_z = 0.9     # 왼쪽 회전
            self.force_turn_end_time = time.time() + 1.2 # 1.2초 동안 강제조향
            self.get_logger().info("FORCE LEFT TURN")

        elif key == 'd':  # 오른쪽 차선 변경 → 오른쪽으로 꺾기
            self.force_turn = True
            self.force_turn_z = -0.9    # 오른쪽 회전
            self.force_turn_end_time = time.time() + 1.2 # 1.2초 동안 강제조향
            self.get_logger().info("FORCE RIGHT TURN")

    def angle_callback(self, msg):
        self.angle_data = msg.data

    def deg_callback(self, msg):
        self.deg_data = msg.data

    def deg_valid_callback(self, msg):
        self.deg_valid = msg.data

    def control_callback(self):
        now = time.time()

        # 강제 조향 모드
        if self.force_turn:
            if now < self.force_turn_end_time:
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = self.force_turn_z
                self.cmd_pub.publish(twist)
                return
            else:
                # 강제 조향 끝남
                self.force_turn = False

        # 평소 차선주행 로직 ===
        z = 0.0

        if self.deg_valid:
            z = -self.k_deg * self.deg_data
        else:
            a = self.angle_data
            a = max(min(a, 120.0), -120.0)
            z = -self.k_angle * a

        alpha = 0.7
        self.z_filtered = alpha * self.z_filtered + (1 - alpha) * z

        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = float(self.z_filtered)
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = MotorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()