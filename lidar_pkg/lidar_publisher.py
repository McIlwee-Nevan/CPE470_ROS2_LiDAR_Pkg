import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from .lidar_data import Lidar_Reader
import numpy as np


class LidarPublisher(Node):

    def __init__(self, lidar):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/lidar', 10)
        
        self.reader = Lidar_Reader(lidar)
        self.time = self.get_clock().now()

    
    def get_scan(self):
        packet = self.reader.get_packet() 
        return packet

    def publish_scan(self):
        self.time = self.get_clock().now()
        packet = self.get_scan()
        points = self.reader.get_points_from_packet(packet)
        angles = points['Angle'].to_numpy()
        angles = angles * -1
        angles = angles % 360
        angle_min = angles[0]
        angle_max = angles[11]
        angle_range = (angle_max - angle_min) % 360
        angle_increment = angle_range / len(angles)
        time_increment = angle_increment / packet['Speed']

        msg = LaserScan()
        msg.header.stamp = self.time.to_msg()
        msg.angle_min = np.deg2rad(angle_min)
        msg.angle_max = np.deg2rad(angle_max)
        msg.angle_increment = np.deg2rad(angle_increment)
        msg.time_increment = time_increment
        msg.range_min = 0.0
        msg.range_max = 1000.0
        ranges = points['Distance (mm)'].to_numpy()
        ranges = ranges.astype(np.float32)
        msg.ranges = ranges

        intensities = points['Intensity'].to_numpy()
        intensities = intensities.astype(np.float32)
        msg.intensities = intensities

        self.publisher_.publish(msg)





def main(args=None):
    rclpy.init(args=args)

    lidar = '/dev/ttyUSB0'
    publisher = LidarPublisher(lidar)

    try:
        while rclpy.ok():
            publisher.publish_scan()
    except KeyboardInterrupt:
        "Shutting down user integer node..."



if __name__ == '__main__':
    main()