import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import numpy as np
import pandas as pd


class FindGap(Node):
    def __init__(self):
        super().__init__('find_gap')
        self.subscription_ = self.create_subscription(
            LaserScan,
            '/lidar',
            self.points_callback,
            10
        )
        self.ranges = []
        self.angles = []
        self.angle_interval = 0


    def points_callback(self, msg: LaserScan):
        new_ranges = msg.ranges
        min_angle = msg.angle_min
        max_angle = msg.angle_max
        if max_angle < min_angle:
            max_angle += 2*np.pi
        new_angles = (np.linspace(min_angle, max_angle, len(new_ranges)))
        for angle in new_angles:
            if angle > (2*np.pi):
                angle -= (2*np.pi)

        self.ranges.append(new_ranges)
        self.angles.append(new_angles)
        self.angle_interval = msg.angle_increment
        
        angles = ' '.join([str(s) for s in new_angles])
        ranges = ' '.join([str(s) for s in new_ranges])
        self.get_logger().info(angles)
        self.get_logger().info(ranges)


def main(args=None):
    rclpy.init(args=args)
    subscriber = FindGap()

    try:
        if rclpy.ok():
            rclpy.spin(subscriber)
    except KeyboardInterrupt:
        "Shutting down user integer node..."


if __name__ == '__main__':
    main()