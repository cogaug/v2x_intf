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

    def _calculate_offsets(self, lat1, lon1, lat2, lon2):
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
        v2xconst.R = 6378137.0 # Radius of the Earth in meters
        x_offset = v2xconst.R * dlon * math.cos(mean_lat)
        y_offset = v2xconst.R * dlat
            
        return int(x_offset*10.0), int(y_offset*10.0) # convert to 0.1 meter unit

    def Recognition2V2XMsg(self, msg):
        # J3224의 sDSMTimeStamp format 구성
        sDSMTimeStamp = (
            msg.vehicle_time[0],  # year
            msg.vehicle_time[1],  # month
            msg.vehicle_time[2],  # day
            msg.vehicle_time[3],  # hour
            msg.vehicle_time[4],  # minute
            msg.vehicle_time[5]*1000+msg.vehicle_time[6], # milliseconds
            9*60 # Timezone in minutes
        )

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
            msg.vehicle_position[0]*1000*1000*10, # Latitude in 1/10th microdegree
            msg.vehicle_position[1]*1000*1000*10  # Longitude in 1/10th microdegree
        )

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
            measurementTimeOffset = (dt2-dt1).total_seconds()*1000 # in milliseconds for MeasurementTimeOffset type
            if measurementTimeOffset > 1500 or measurementTimeOffset < -1500 : # sDSMTimeStamp보다 1.5초 빨리 디텍트한 객체
              continue
            
            object_id = msg.vehicle_id << 16 + idx  # vehicle_id는 제어부에서 임의로 설정되는데 현재 3대의 자율차에 1,2,3으로 할당.

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
              v2xconst.fDetectedObjectCommonData,
              obj.object_class,
              int(obj.object_accuracy*100),
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

        # Convert header to C struct data type
        # Ref : v2x_intf_hdr_type
        hdr_data = struct.pack(
            v2xconst.fmsgHdrType,
            v2xconst.HDR_FLAG,          # hdr
            v2xconst.MSG_RECOGNITION,   # msgID for recognition
            len(packed_data)            # msgLen
        )

        return hdr_data+packed_data

    def recognition_callback(self, msg):
        self.get_logger().info('Received recognition message')
        try:
            data = self.Recognition2V2XMsg(msg)
            # Send the received message data to the server over the shared TCP connection
            response = self.connection_manager.send_data(data)
            if response:
                self.get_logger().info(f'Received response from server: {response}')
        except Exception as e:
            self.get_logger().error(f'Error processing recognition message: {e}')
