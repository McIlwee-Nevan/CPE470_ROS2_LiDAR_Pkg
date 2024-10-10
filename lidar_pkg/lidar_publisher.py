import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from lidar_data import Lidar_Reader


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
        scan_time = self.time
        self.time = self.get_clock().now()
        scan_time = self.time - scan_time
        packet = self.get_scan()
        msg = LaserScan()
        msg.header.stamp = self.time


def main(args=None):
    rclpy.init(args=args)

    lidar = '/dev/ttyUSB0'
    publisher = LidarPublisher(lidar)

    try:
        while rclpy.ok():
            publisher.run()
    except KeyboardInterrupt:
        "Shutting down user integer node..."



if __name__ == '__main__':
    main()