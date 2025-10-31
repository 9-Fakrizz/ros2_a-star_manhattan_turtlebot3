#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # ✅ ต้องมี



class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # ✅ match LiDAR
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )

        self.get_logger().info("✅ LiDAR tester started — waiting for /scan data...")
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.scan_plot = None


    def lidar_callback(self, msg):
        angles = []
        distances = []

        angle = msg.angle_min
        for r in msg.ranges:
            # กรองค่าไม่ใช่ตัวเลขและค่าเกินช่วง
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            # ✅ เก็บเฉพาะระยะที่ <= 0.6 m
            if msg.range_min < r < msg.range_max and r <= 0.6:
                angles.append(angle)
                distances.append(r)

            angle += msg.angle_increment

        # ถ้าไม่มีค่า valid
        if not distances:
            return

        # ล้าง plot เก่าแล้ว plot ใหม่
        self.ax.clear()
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_title("LiDAR (≤0.6 m points only)")
        self.ax.scatter(angles, distances, s=8, c='red')
        self.ax.set_rlim(0, 1.0)
        plt.pause(0.01)

        self.get_logger().info(f"Plotted {len(distances)} valid LiDAR points ≤0.6 m")


def main(args=None):
    rclpy.init(args=args)
    node = LidarTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("LiDAR test stopped.")
    finally:
        plt.ioff()
        plt.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
