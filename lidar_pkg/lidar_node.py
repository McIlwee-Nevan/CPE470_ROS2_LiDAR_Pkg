import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from lidar_data.msg import LidarData

import copy
import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN
#from matplotlib import pyplot as plt





class FindGap(Node):
    def __init__(self, a=-1.0):
        super().__init__('find_gap')
        self.subscription_ = self.create_subscription(
            LaserScan,
            '/lidar',
            self.points_callback,
            10
        )
        self.subscription_ = self.create_subscription(
            Bool,
            '/enable_localization',
            self.enable_localization,
            10
        )
        self.publisher_ = self.create_publisher(
            LidarData,
            '/lidar_data',
            10
        )

        #self.timer = self.create_timer(0.1, self.find_gap)
        self.timer = self.create_timer(0.1, self.publish_lidar_data)

        self.angles = []  
        self.ranges = []
        self.next_angles = []
        self.next_ranges = []
        self.angle_interval = -1.0

        self.localization = False

        self.nearest_angle = -1.0
        self.nearest_range = -1.0
        self.dist_in_front = -1.0

        self.corner_angles = np.ones(4) * -1
        self.corner_ranges = np.ones(4) * -1
        
        self.x = -1.0
        self.y = -1.0
        self.theta = -1.0

        '''
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        plt.show()
        '''

    def enable_localization(self, msg: Bool):
        self.localization = msg.data
        if not self.localization:
            self.corner_angles.fill(-1.0)
            self.corner_ranges.fill(-1.0)
            self.x = -1.0
            self.y = -1.0
            self.theta = -1.0
    
    def points_callback(self, msg: LaserScan):
        new_ranges = msg.ranges
        min_angle = msg.angle_min
        max_angle = msg.angle_max
        if max_angle < min_angle:
            max_angle += 2.0*np.pi
        new_angles = np.linspace(min_angle, max_angle, len(new_ranges))
        for i in range(len(new_angles)):
            if new_angles[i] > (2*np.pi):
                new_angles[i] -= (2*np.pi)

        self.next_angles.extend(new_angles)
        self.next_ranges.extend(new_ranges)
        self.angle_interval = msg.angle_increment

        min_num_pts = 2000
        
        if len(self.next_angles) > min_num_pts:
            self.next_angles = np.array(self.next_angles)
            self.next_ranges = np.array(self.next_ranges)
            #Sort
            index_array = np.argsort(self.next_angles)
            self.next_angles = np.take_along_axis(self.next_angles, index_array, 0)
            self.next_ranges = np.take_along_axis(self.next_ranges, index_array, 0)
            
            #DBSCAN to remove outliers
            clean_angles, clean_ranges, clusters = self.clean_data(self.next_angles, self.next_ranges)

            self.angles = np.copy(clean_angles)
            self.ranges = np.copy(clean_ranges)
            self.next_angles = []
            self.next_ranges = []

            #Update Info
            self.nearest_angle = self.angles[self.ranges.argmin()]
            self.nearest_range = self.ranges.min()
            self.dist_in_front = self.ranges[0]

            if self.localization:
                #Filter room cluster
                room_cluster = clusters[self.ranges.argmin()]
                
                #self.get_logger().info(f"room: {len(clusters[clusters == room_cluster])}:{len(clusters)}")
                room_angles = self.angles[clusters == room_cluster]
                room_ranges = self.ranges[clusters == room_cluster]
                
                self.update_pos(room_angles, room_ranges)                

    
    def clean_data(self, angles, ranges):
        y = np.array(ranges * np.sin(angles))
        x = np.array(ranges * np.cos(angles))
        data = np.column_stack((x,y))
        cluster = DBSCAN(eps=0.075, min_samples=5)
        clustering = cluster.fit(data)
        labels = clustering.labels_
        
        #self.get_logger().info(f"outliers {len(labels[labels == -1])}:{len(labels)}")

        return angles[labels >= 0], ranges[labels >= 0], labels[labels >= 0]

    def update_pos(self, angles, ranges):
        corner_angles = np.zeros(4)
        corner_ranges = np.zeros(4)

        #get corners
        corner_angles, corner_ranges = self.get_corners(angles, ranges)
        
        self.get_logger().info(f"Angles, Ranges")
        self.get_logger().info(f"{np.column_stack((corner_angles, corner_ranges))}")
        self.corner_angles = corner_angles
        self.corner_ranges = corner_ranges

        #update x, y, theta
        angle_ab = (self.corner_angles[1]-self.corner_angles[0])
        angle_ab = angle_ab if (angle_ab >= 0) else (angle_ab + (2*np.pi))
        alpha = np.arcsin((self.corner_ranges[1] * np.sin(angle_ab))/1.15)
        theta = np.pi - self.corner_angles[0] + alpha
        theta = theta if (theta >= 0) else (theta + (2*np.pi))

        self.x = self.corner_ranges[0] * np.cos(alpha)
        self.y = self.corner_ranges[0] * np.sin(alpha)
        self.theta = theta

    def get_corners(self, angles, ranges):
        #smooth
        window = 5
        padding = int((window - 1)/2)
        padded_ranges = np.concatenate((ranges[0-padding:], ranges, ranges[:padding]))
        smoothed_ranges = np.array([np.average(padded_ranges[i:i+window]) for i in range(len(ranges))])
        
        dr_dt = np.diff(smoothed_ranges)
        dr_dt = np.concatenate((dr_dt, dr_dt[:1]))
        padded_dr_dt = np.concatenate((dr_dt[0-padding:], dr_dt, dr_dt[:padding]))
        smoothed_dr_dt = np.array([np.average(padded_dr_dt[i:i+window]) for i in range(len(dr_dt))])
        '''
        possible_corners = []
        i = 0
        sign = smoothed_dr_dt[i]
        while i < len(smoothed_dr_dt):
            if sign > 0:
                if smoothed_dr_dt[i] < 0:
                    possible_corners.append(i)
                    sign = smoothed_dr_dt[i]
                    i+=10
                else:
                    i+=1
            elif sign < 0:
                if smoothed_dr_dt[i] > 0:
                    sign = smoothed_dr_dt[i]
                    i+=10
                else:
                    i+=1

        possible_corners = np.array(possible_corners)
        '''
        '''
        d2r_dt2 = np.diff(smoothed_dr_dt)
        d2r_dt2 = np.concatenate((d2r_dt2, d2r_dt2[:1]))
        padded_d2r_dt2 = np.concatenate((d2r_dt2[0-padding:], d2r_dt2, d2r_dt2[:padding]))
        d2r_dt2_smooth = np.array([np.average(padded_d2r_dt2[i:i+window]) for i in range(len(d2r_dt2))])

        padded_min = np.concatenate((d2r_dt2_smooth[0-50:], d2r_dt2_smooth, d2r_dt2_smooth[:50]))
        rolling_min = np.array([padded_min[i:i+101].min() for i in range(len(d2r_dt2_smooth))])
        threshold = np.percentile(d2r_dt2_smooth, 1)

        # Discarding borderline inlexion points
        inflexion_points = (d2r_dt2_smooth <= rolling_min) & (d2r_dt2_smooth < threshold)
        possible_corners = np.array(range(len(ranges)))
        possible_corners = possible_corners[inflexion_points]
        self.get_logger().info(f"{possible_corners}")

        cyclic_corners = np.concatenate((possible_corners, possible_corners[:1]))
        y = np.array(ranges[cyclic_corners] * np.sin(angles[cyclic_corners]))
        x = np.array(ranges[cyclic_corners] * np.cos(angles[cyclic_corners]))
        slopes = np.arctan2( # calculating slope of each line connecting corners
                np.diff(y),
                np.diff(x)
        )
        cyclic_slopes = np.concatenate((slopes[-1:], slopes))
        slope_diffs = abs(np.diff(cyclic_slopes))
        corners = possible_corners[slope_diffs > 5e-1]

        corner_angles = angles[corners]
        corner_ranges = ranges[corners]

        self.ax.clear()
        self.ax.scatter(angles, smoothed_ranges)
        self.ax.scatter(angles[possible_corners], ranges[possible_corners], c='b')
        self.ax.scatter(corner_angles, corner_ranges, c='r')
        self.ax.set_rmax(smoothed_ranges.max() * 1.1)
        self.ax.set_title("LiDAR Sensor Data")
        plt.draw()
        plt.pause(0.001)
        


        '''
        if self.corner_angles[0] < 0: #No previous knowledge about corner A, assume robot is facing north aruco, corner A is first in the array
            fourth = np.pi / 2
            corners = np.array([np.argmax(smoothed_ranges[(angles >= i*fourth) & (angles < (i+1)*fourth)]) + len(smoothed_ranges[angles < i*fourth]) for i in range(4)])    
            corner_angles = angles[corners]
            corner_ranges = smoothed_ranges[corners]
        else:
            corner_angles = np.zeros(4)
            corner_ranges = np.zeros(4)
            for i in range(4):
                angle_max = self.corner_angles[i] + (0.1 * np.pi)
                angle_min = self.corner_angles[i] - (0.1 * np.pi)
                filtered_angles = []
                filtered_ranges = []
                if angle_max > 2*np.pi:
                    filtered_angles = angles[(angles >= angle_min) | (angles < angle_max - (2*np.pi))]
                    filtered_ranges = smoothed_ranges[(angles >= angle_min) | (angles < angle_max - (2*np.pi))]
                elif angle_min < 0:
                    filtered_angles = angles[(angles >= (angle_min + (2*np.pi))) | (angles < angle_max)]
                    filtered_ranges = smoothed_ranges[(angles >= (angle_min + (2*np.pi))) | (angles < angle_max)]
                else:
                    filtered_angles = angles[(angles >= angle_min) & (angles < angle_max)]
                    filtered_ranges = smoothed_ranges[(angles >= angle_min) & (angles < angle_max)]
                
                if len(filtered_ranges) > 0:
                    index = np.argmax(filtered_ranges)
                    corner_angles[i] = filtered_angles[index]
                    corner_ranges[i] = filtered_ranges[index]
        '''
        self.ax.clear()
        self.ax.scatter(angles, smoothed_ranges)
        #self.ax.scatter(angles[possible_corners], ranges[possible_corners], c='b')
        self.ax.scatter(corner_angles, corner_ranges, c='r')
        self.ax.set_rmax(smoothed_ranges.max() * 1.1)
        self.ax.set_title("LiDAR Sensor Data")
        plt.draw()
        plt.pause(0.001)
        '''

        if np.all(corner_angles == 0):
            return self.corner_angles, self.corner_ranges
        return corner_angles, corner_ranges

    def publish_lidar_data(self):
        '''
        self.get_logger().info(f"Nearest Point's Angle: {self.angle_closest:0.3f} Radians")
        self.get_logger().info(f"Distance in Front: {self.dist_in_front:0.3f} Meters")
        '''
        msg = LidarData()
        msg.nearest_angle = self.nearest_angle
        msg.nearest_range = self.nearest_range
        msg.range_in_front = self.dist_in_front
        msg.theta = self.theta
        msg.x = self.x
        msg.y = self.y
    
        self.publisher_.publish(msg)

    def find_gap(self):
        if self.angle_interval < 0:
            self.get_logger().warn("Waiting for lidar publisher...")
            return
        
        min_num_pts = (np.pi * 2) / self.angle_interval
        if len(self.angles) < min_num_pts:
            self.get_logger().info("Not enough points, waiting 1 second...")
            return

        
        #filter points too far away
        keep = self.ranges < (1.5 * np.median(self.ranges))
        filtered_angles = self.angles[keep]
        filtered_ranges = self.ranges[keep]
        
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