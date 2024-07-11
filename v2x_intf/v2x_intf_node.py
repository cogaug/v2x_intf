import rclpy as rclpy
from rclpy.node import Node

from v2x_msgs.msg import Recognition, Objects
import socket
import threading
import select
import struct
import datetime
import math


R = 6378137.0 # Radius of the Earth in meters
equipmentType = 2 # unknown (0), rsu (1), obu (2)

class TcpConnectionManager:
    def __init__(self):

        self.obu_ip = '192.168.2.100' # TODO : MATCH IT!
        self.obu_port = 9201

        self.obu_connected = False
        self.receive_buffer = b''
        self.lock = threading.Lock()
        self.client_socket = None

        try :
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.obu_ip, self.obu_port))
            self.obu_connected = True
        except Exception as e:
            print('Error:', str(e))
            self.obu_connected = False

    def send_data(self, data):
        if self.obu_connected :
            with self.lock:
                try:
                    # Serialize and send the data to the server
                    serialized_data = self.serialize_data(data)  # Implement serialization function
                    self.client_socket.send(serialized_data)
                    
                except Exception as e:
                    print('Error:', str(e))
                    return None


    def receive_data(self):
        with self.lock:
            ready_to_read, _, _ = select.select([self.client_socket], [], [], 0.1)
            if ready_to_read:
                received_data = self.client_socket.recv(1024)
                self.response_buffer += received_data
                return self.response_buffer.decode()
            return None

    def close_connection(self):
        with self.lock:
            self.client_socket.close()

    def serialize_data(self, data):
        # Implement serialization logic for your data here
        # For example, you can use pickle or another serialization method
        # Return the serialized data as bytes
        return bytes(str(data), 'utf-8')


class RecognitionSubscriber(Node):
    def __init__(self, connection_manager):
        super().__init__('recognition_subscriber')
        self.subscription = self.create_subscription(
            Recognition,
            'v2x_msgs/recognition',
            self.recognition_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.connection_manager = connection_manager


    def _calculate_offsets(lat1, lon1, lat2, lon2):
      # Convert degrees to radians
      lat1_rad = math.radians(lat1)
      lon1_rad = math.radians(lon1)
      lat2_rad = math.radians(lat2)
      lon2_rad = math.radians(lon2)
        
      # Differences in coordinates
      dlat = lat2_rad - lat1_rad
      dlon = lon2_rad - lon1_rad
        
      # Mean latitude
      mean_lat = (lat1_rad + lat2_rad) / 2
        
      # Calculate offsets
      x_offset = R * dlon * math.cos(mean_lat)
      y_offset = R * dlat
        
      return int(x_offset*10.0), int(y_offset*10.0) # convert to 0.1 meter unit

    def Recognition2V2XMsg(self, msg):
        sDSMTimeStamp = (
            msg.vehicle_time[0],  # year
            msg.vehicle_time[1],  # month
            msg.vehicle_time[2],  # day
            msg.vehicle_time[3],  # hour
            msg.vehicle_time[4],  # minute
            msg.vehicle_time[5]*1000+msg.vehicle_time[6], # milliseconds
            9*60 # Timezone in minutes
        )
        dt1 = datetime(
            msg.datetime[0],  # year
            msg.datetime[1],  # month
            msg.datetime[2],  # day
            msg.datetime[3],  # hour
            msg.datetime[4],  # minute
            msg.datetime[5],  # second
            msg.datetime[6]*1000   # milliseconds
        )

        position3D = (
            msg.vehicle_pos[0]*1000*1000*10, # Latitude in 1/10th microdegree
            msg.vehicle_pos[1]*1000*1000*10  # Longitude in 1/10th microdegree
        )

        positionAccuracy = (
            255,  # semiMajor
            255,  # semiMinor
            65535 # orientation
        )

        # Convert msg to C struct data type
        fDDateTimeType = 'HBBBBHh' # (year, month, day, hour, minute, second, offset)
        fPosition3D = 'll' # (latitude, longitude)
        fPositionalAccuracy = 'BBH'
        fDetectedObjectCommonData = '<BBHhBhhBHBHB'
        fFirstPart = f'<B {fDDateTimeType} {fPosition3D} {fPositionalAccuracy} B' # equipmentType, sDSMTimeStamp, refPos, refPosXYConf, numDetectedObjects

        packed_objects = b''
        num_object = 0
        for idx, obj in enumerate(msg.object_data) :
            # Create datetime objects, including milliseconds
            dt2 = (
              obj.datetime[0],  # year
              obj.datetime[1],  # month
              obj.datetime[2],  # day
              obj.datetime[3],  # hour
              obj.datetime[4],  # minute
              obj.datetime[5],  # second
              obj.datetime[6]*1000  # millisecond to microsecond
            )
            measurementTime = (dt2-dt1).total_seconds()*1000
            if measurementTime > 1500 or measurementTime < -1500 : # sDSMTimeStamp보다 1.5초 빨리 디텍트한 객체
              continue
            
            object_id = msg.vehicle_id << 16 + idx

            offsetX, offsetY = self._calculate_offsets( msg.vehicle_pos[0],  msg.vehicle_pos[1], obj.object_position[0], obj.object_position[1])
            if offsetX > 32767 or offsetX < -32767 or offsetY > 32767 or offsetY < -32767 :
                continue
            
            speed = int(obj.object_velocity / 0.02)
            if speed > 8191 :
                continue
            
            if obj.object_heading < 0.0 :
                obj.object_heading += 360.0
            heading = int(((obj.object_heading)%360.0)/0.0125)  # in 0.0125 degree unit
            if heading > 28800 :
                continue
            
            packed_object = struct.pack(
              fDetectedObjectCommonData,
              obj.object_class,
              int(obj.object_accuracy*100),
              object_id,
              measurementTime,
              0, # timeConfidence
              offsetX, offsetY,
              0, # posConfidence
              speed,
              0, # speedConfidence
              heading,
              0 # headingConfidence
            )
            packed_objects += packed_object
            num_object += 1

        packed_data = struct.pack(
          fFirstPart,
          equipmentType,
          *sDSMTimeStamp,
          *position3D,
          *positionAccuracy,
          num_object
        )
        packed_data += packed_objects

        return packed_data

    def recognition_callback(self, msg):
        print('Received recognition message:', msg)
        data = self.Recognition2V2XMsg(msg)
        # Send the received message data to the server over the shared TCP connection
        response = self.connection_manager.send_data(data)
        if response:
            print('Received from server:', response)


def main(args=None):
    rclpy.init(args=args)
    connection_manager = TcpConnectionManager()
    recognition_subscriber = RecognitionSubscriber(connection_manager)
    # TODO Add other nodes

    # received_data = connection_manager.receive_data()
    # if received_data is not None:
    #     print('Received from server:', received_data)

    # rclpy.spin(recognition_subscriber)


    try:
        node = rclpy.create_node('main_node')  # Create a main node instance
        # Main loop for your node
        while rclpy.ok():
            # Receive data from the server asynchronously
            if connection_manager.obu_connected:
                received_data = connection_manager.receive_data()
                if received_data is not None:
                    print('Received from server:', received_data)

            # TODO: Process other tasks or messages here
            rclpy.spin_once(recognition_subscriber)  # Spin ROS 2 for a single iteration
    
    except KeyboardInterrupt:
        pass

    # Close the connection when done
    connection_manager.close_connection()

    # TODO Add other nodes to destroy_node
    recognition_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()