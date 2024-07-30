import rclpy
from rclpy.node import Node
from v2x_msgs.msg import Recognition
import struct
import datetime
import math
from v2x_intf_pkg.v2x_const import V2XConstants as v2xconst

class RecognitionSubscriber(Node):
    def __init__(self, connection_manager):
        super().__init__('recognition_subscriber')

        self.equipmentType = 2 # unknown (0), rsu (1), obu (2)

        self.subscription = self.create_subscription(
            Recognition,
            'v2x_msgs/recognition',
            self.recognition_callback,
            10
        )
        
        # self.subscription  # prevent unused variable warning
        self.connection_manager = connection_manager

    def Recognition2V2XMsg(self, msg):
        # J3224의 sDSMTimeStamp format 구성
        sDSMTimeStamp = (
            msg.vehicle_time[0],  # year
            msg.vehicle_time[1],  # month
            msg.vehicle_time[2],  # day
            msg.vehicle_time[3],  # hour
            msg.vehicle_time[4],  # minute
            msg.vehicle_time[5]*1000+(msg.vehicle_time[6]//1000), # milliseconds
            9*60 # Timezone in minutes
        )
        self.get_logger().info(f'msg.vehicle_time: {msg.vehicle_time}')
        self.get_logger().info(f'--> sDSMTimeStamp: {sDSMTimeStamp}')


        # Create datetime objects, including milliseconds to calculate measurementTimeOffset
        dt1 = datetime.datetime(
            msg.vehicle_time[0],  # year
            msg.vehicle_time[1],  # month
            msg.vehicle_time[2],  # day
            msg.vehicle_time[3],  # hour
            msg.vehicle_time[4],  # minute
            msg.vehicle_time[5],  # second
            msg.vehicle_time[6]   # microsecond
        )

        position3D = (
            int(msg.vehicle_position[0]*1000*1000*10), # Latitude in 1/10th microdegree
            int(msg.vehicle_position[1]*1000*1000*10)  # Longitude in 1/10th microdegree
        )
        self.get_logger().info(f'msg.vehicle_position: {msg.vehicle_position}')
        self.get_logger().info(f'--> position3D: {position3D}')
        if position3D[0] > 900000000 or position3D[0] < -900000000 :
            self.get_logger().info(f'--> Latitude is out of range')
            return None
        
        if position3D[1] > 1800000000 or position3D[1] < -1800000000 :
            self.get_logger().info(f'--> Longitude is out of range')
            return None

        positionAccuracy = (
            255,  # semiMajor
            255,  # semiMinor
            65535 # orientation
        )

        packed_objects = b''
        num_object = 0
        for idx, obj in enumerate(msg.object_data) :
            # Create datetime objects, including milliseconds to calculate measurementTimeOffset
            dt2 = datetime.datetime(
              obj.detection_time[0],  # year
              obj.detection_time[1],  # month
              obj.detection_time[2],  # day
              obj.detection_time[3],  # hour
              obj.detection_time[4],  # minute
              obj.detection_time[5],  # second
              obj.detection_time[6]   # microsecond
            )
            self.get_logger().info(f'obj.detection_time: {obj.detection_time}')

            measurementTimeOffset = int((dt2-dt1).total_seconds()*1000) # in milliseconds for MeasurementTimeOffset type # it should have -1500 ~ 1500 in 1ms unit (-1.5 sec ~ 1.5 sec)
            self.get_logger().info(f'--> (dt2-dt1).total_seconds(): {(dt2-dt1).total_seconds()}')
            self.get_logger().info(f'--> measurementTimeOffset: {measurementTimeOffset}')
            
            if measurementTimeOffset > 1500 or measurementTimeOffset < -1500 : # sDSMTimeStamp보다 1.5초 빨리 디텍트한 객체
              self.get_logger().info(f'--> measurementTimeOffset is out of range')
              continue
            
            object_id = msg.vehicle_id << 8 + idx  # vehicle_id는 제어부에서 임의로 설정되는데 현재 3대의 자율차에 1,2,3으로 할당.
            self.get_logger().info(f'--> object_id: {object_id:#X}')
            
            offsetX = int(obj.object_position[0]*10)
            offsetY = int(obj.object_position[1]*10)
            self.get_logger().info(f'obj.object_position {obj.object_position}')
            self.get_logger().info(f'--> offsetX: {offsetX}, offsetY: {offsetY}')

            if offsetX > 32767 or offsetX < -32767 or offsetY > 32767 or offsetY < -32767 :
                self.get_logger().info(f'--> offsetX or offsetY is out of range')
                continue
            
            speed = int(obj.object_velocity / 0.02)
            self.get_logger().info(f'obj.object_velocity: {obj.object_velocity}')
            self.get_logger().info(f'--> speed: {speed}')

            if speed > 8191 :
                self.get_logger().info(f'--> speed is out of range')
                speed = 8192 # represents "speed is unavailable"
            
            if obj.object_heading < 0.0 :
                obj.object_heading += 360.0
            heading = int(((obj.object_heading)%360.0)/0.0125)  # in 0.0125 degree unit
            self.get_logger().info(f'obj.object_heading: {obj.object_heading}')
            self.get_logger().info(f'--> heading: {heading}')
            if heading > 28800 :
                self.get_logger().info(f'--> heading is out of range')
                continue
                      
            packed_object = struct.pack(
              v2xconst.fDetectedObjectCommonData,
              obj.object_class,
              int(obj.recognition_accuracy),
              object_id,
              measurementTimeOffset,
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
          v2xconst.fFirstPart,
          self.equipmentType,
          *sDSMTimeStamp,
          *position3D,
          *positionAccuracy,
          num_object
        )
        packed_data += packed_objects

        self.get_logger().info(f'Packed data: {packed_data}, length : {len(packed_data)}')

        # Convert header to C struct data type
        # Ref : v2x_intf_hdr_type
        hdr_data = struct.pack(
            v2xconst.fmsgHdrType,
            v2xconst.HDR_FLAG,          # hdr
            v2xconst.MSG_RECOGNITION,   # msgID for recognition
            len(packed_data)            # msgLen
        )

        self.get_logger().info(f'Header data: {hdr_data}, length : {len(hdr_data)}')
        self.get_logger().info(f'HDR_FLAG: {v2xconst.HDR_FLAG}')
        self.get_logger().info(f'MSG_RECOGNITION: {v2xconst.MSG_RECOGNITION}')

        return hdr_data+packed_data

    def recognition_callback(self, msg):
        self.get_logger().info('Received recognition message')
        try:
            data = self.Recognition2V2XMsg(msg)
            # Send the received message data to the server over the shared TCP connection
            if data :
              response = self.connection_manager.send_data(data)
              if response:
                  self.get_logger().info(f'Received response from server: {response}')
        except Exception as e:
            self.get_logger().error(f'Error processing recognition message: {e}')
