import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data


class FrontLidar(Node):
    def __init__(self):
        super().__init__('front_lidar')

        # /scan 구독 (라이다 센서 QoS)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        # 정면 거리 [m] 퍼블리시하는 /m 토픽
        self.front_pub = self.create_publisher(
            Float32,
            '/m',
            10
        )

        self.get_logger().info('FrontLidar node started. Subscribing /scan, publishing /m')

    def lidar_callback(self, msg: LaserScan):
        # TurtleBot3 LDS-02 기준 0번 인덱스가 정면
        front = msg.ranges[0]

        # inf 처리 (장애물 없으면 아주 멀리 있다고 가정)
        if math.isinf(front):
            front = 10.0

        # 로그로 확인
        self.get_logger().info(f'front = {front:.2f} m')

        # /m 토픽으로 발행
        m_msg = Float32()
        m_msg.data = float(front)
        self.front_pub.publish(m_msg)


def main():
    rclpy.init()
    node = FrontLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()