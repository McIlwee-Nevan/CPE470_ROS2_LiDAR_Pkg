import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import numpy as np


class FindGap(Node):
    def __init__(self):
        super().__init__('find_gap')
        #self.create_subscription




def main(args=None):
    rclpy.init(args=args)

    try:
        if rclpy.ok():
            pass
    except KeyboardInterrupt:
        "Shutting down user integer node..."



if __name__ == '__main__':
    main()