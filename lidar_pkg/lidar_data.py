import serial
import struct
import numpy as np
import pandas as pd



class Lidar_Reader():
    def __init__(self, lidar):
        self.ser = serial.Serial(lidar, 230400)
        self.packet_indeces = ['Header', 'VerLen', 'Speed', 'Start Angle', 
                'P1 Dist', 'P1 Intensity',
                'P2 Dist', 'P2 Intensity',
                'P3 Dist', 'P3 Intensity',
                'P4 Dist', 'P4 Intensity',
                'P5 Dist', 'P5 Intensity',
                'P6 Dist', 'P6 Intensity',
                'P7 Dist', 'P7 Intensity',
                'P8 Dist', 'P8 Intensity',
                'P9 Dist', 'P9 Intensity',
                'P10 Dist', 'P10 Intensity',
                'P11 Dist', 'P11 Intensity',
                'P12 Dist', 'P12 Intensity',
                'End Angle', 'Timestamp', 'CRC Check']
        self.parser = struct.Struct('<BBHHHBHBHBHBHBHBHBHBHBHBHBHBHHB') #Converts byte string into tuple of objects, following the format in the Lidar documentation

    def get_byte_packet(self):
        header = self.ser.read()
        while header != b'\x54':
            header = self.ser.read()

        packet = self.ser.read(46)
        packet = header + packet
        return packet

    def parse_byte_packet(self, bytes):
        return self.parser.unpack(bytes)
    
    def get_packet(self):
        data = self.get_byte_packet()
        data = self.parse_byte_packet(data)
        packet = pd.Series(data, self.packet_indeces, dtype='int64')
        return packet
    
    def get_points_from_packet(self, packet=None):
        if packet is None:
            packet = self.get_packet()

        points = pd.DataFrame()

        dist = pd.Series(packet[['P1 Dist',
                'P2 Dist',
                'P3 Dist',
                'P4 Dist',
                'P5 Dist',
                'P6 Dist',
                'P7 Dist',
                'P8 Dist',
                'P9 Dist',
                'P10 Dist',
                'P11 Dist',
                'P12 Dist']].to_numpy())
        
        intensities = pd.Series(packet[['P1 Intensity',
                'P2 Intensity',
                'P3 Intensity',
                'P4 Intensity',
                'P5 Intensity',
                'P6 Intensity',
                'P7 Intensity',
                'P8 Intensity',
                'P9 Intensity',
                'P10 Intensity',
                'P11 Intensity',
                'P12 Intensity']].to_numpy())
        
        points['Distance (mm)'] = dist
        points['Intensity'] = intensities

        start_angle = float(packet['Start Angle']) / 100
        end_angle = float(packet['End Angle']) / 100

        if end_angle < start_angle:
            end_angle += 360

        points['Angle'] = (np.linspace(start_angle, end_angle, 12)) % 360

        return points
    
    def get_points(self, time_interval):
        #time_interval: seconds
        if time_interval >= 30:
            raise(ValueError)
        packet = self.get_packet()
        start_time = packet['Timestamp']
        current_time = start_time
        points = pd.DataFrame(data=self.get_points_from_packet(packet))

        while ((current_time - start_time) % 30000) < (time_interval * 1000):
            packet = self.get_packet()
            current_time = packet['Timestamp']
            
            temp = self.get_points_from_packet(packet)
            points = points._append(temp, ignore_index=True)

        return points

        
    
    def __del__(self):
        self.ser.close()


def main():
    lidar = '/dev/ttyUSB0'
    reader = Lidar_Reader(lidar)

    for i in range(4):
        print(reader.get_packet())

    points = reader.get_points(0.025)
    ranges = points['Distance (mm)'].to_numpy()
    ranges = ranges.astype(np.float32)
    print(ranges)

        

if __name__ == '__main__':
    main()