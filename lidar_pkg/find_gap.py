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
        self.timer = self.create_timer(1, self.find_gap)
        self.ranges = []
        self.angles = []
        self.angle_interval = -1


    def points_callback(self, msg: LaserScan):
        new_ranges = msg.ranges
        min_angle = msg.angle_min
        max_angle = msg.angle_max
        if max_angle < min_angle:
            max_angle += 2*np.pi
        new_angles = np.linspace(min_angle, max_angle, len(new_ranges))
        for i in range(len(new_angles)):
            if new_angles[i] > (2*np.pi):
                new_angles[i] -= (2*np.pi)

        self.ranges.extend(new_ranges)
        self.angles.extend(new_angles)
        self.angle_interval = msg.angle_increment
        
        """ 
        angles = ' '.join([str(s) for s in new_angles])
        ranges = ' '.join([str(s) for s in new_ranges])
        self.get_logger().info(angles)
        self.get_logger().info(ranges)
        """

    def find_gap(self):
        if self.angle_interval < 0:
            self.get_logger().warn("Waiting for lidar publisher...")
            return
        
        min_num_pts = (np.pi * 2) / self.angle_interval
        if len(self.angles) < min_num_pts:
            self.get_logger().info("Not enough points, waiting 1 second...")
            return


        #sort polar points by ranges
        angles = np.copy(self.angles)
        ranges = np.copy(self.ranges)
        self.ranges = []
        self.angles = []
        index_array = np.argsort(angles)
        sorted_angles = np.take_along_axis(angles, index_array, 0)
        sorted_ranges = np.take_along_axis(ranges, index_array, 0)
        
        #filter points too far away
        keep = sorted_ranges < 1.5 * np.median(sorted_ranges)
        filtered_angles = sorted_angles[keep]
        filtered_ranges = sorted_ranges[keep]
        
        #find gap in angle
        ends = []

        for i in range(len(filtered_angles)):
            prev = filtered_angles[i-1]
            if prev > filtered_angles[i]:
                prev -= 2*np.pi
    
            if abs(prev - filtered_angles[i]) > 2*self.angle_interval:
                ends = [i-5, (i+5)%len(filtered_angles)]
                break
        
        if not ends:
            self.get_logger().warn("Could not find gap, trying again...")
            return

        x = filtered_ranges[ends] * np.cos(filtered_angles[ends])
        y = filtered_ranges[ends] * np.sin(filtered_angles[ends])

        self.get_logger().info(f"Angles: ({filtered_angles[ends]})")
        self.get_logger().info(f"Ranges: ({filtered_ranges[ends]})")
        self.get_logger().info(f"Point 1: {x[0]:.4f}, {y[0]:.4f}")
        self.get_logger().info(f"Point 2: {x[1]:.4f}, {y[1]:.4f}")

        

        
        """ 
        #Find gap
        window_size = int(0.1 * len(sorted_angles))
        ends = []

        for i in range(len(sorted_angles)):
            if abs(sorted_ranges[i] - sorted_ranges[i-1]) > 0.5:
                if abs(np.median(sorted_ranges[i-len(sorted_angles):i-len(sorted_angles)+window_size]) - np.median(sorted_ranges[i-window_size:i])) > 0.5:
                    ends.append(i if sorted_ranges[i] < sorted_ranges[i-1] else (i-1))

        if len(ends) > 2:
            #check for false ends
            interval = []
            if ends[i-1] > ends[i]:
                interval = [ends[i-1]-len(sorted_ranges), ends[i]]
            else:
                interval = [ends[i-1], ends[i]]

            """ 
        



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