# This file to add the publish function for the v2x interface messages.
import rclpy
from rclpy.node import Node
from v2x_msgs.msg import Recognition, Objects
from v2x_intf_pkg.v2x_const import V2XConst as v2xconst
import struct

class RecognitionPublisher(Node):
    def __init__(self):
        super().__init__('recognition_publisher')

    def _proc_recognition_msg(self, data) : # data contains main message and object data
        main_msg_size = struct.calcsize(v2xconst.fFirstPart)
        main_msg_data = data[: main_msg_size]
        (equipmentType, year, month, day, hour, minute, second, offset,
         latitude, longitude, semiMajor, semiMinor, orientation, numDetectedObjects) = struct.unpack(fFirstPart, main_msg_data)
        
        self.get_logger().info('Equipment Type: %d' % equipmentType)

        vehicle_time = [year, month, day, hour, minute, second // 1000, second % 1000]
        vehicle_pos = [latitude / (1000 * 1000 * 10), longitude / (1000 * 1000 * 10)]

        # Unpack detected objects
        fDetectedObjectCommonData = '<BBHhBhhBHBHB'
        object_data_list = []
        for i in range(numDetectedObjects):
            obj_data_start = main_msg_size + i * struct.calcsize(fDetectedObjectCommonData)
            obj_data_end = obj_data_start + struct.calcsize(fDetectedObjectCommonData)
            obj_data = data[obj_data_start:obj_data_end]

            (obj_class, obj_accuracy, object_id, measurementTime,
             timeConfidence, offsetX, offsetY, posConfidence,
             speed, speedConfidence, heading, headingConfidence) = struct.unpack(fDetectedObjectCommonData, obj_data)

            object_datetime = vehicle_time[:6] + [0]  # Assuming milliseconds are zero as it's not provided
            object_position = [
                vehicle_pos[0] + offsetX / (10 * 6378137.0 * math.cos(math.radians(vehicle_pos[0]))),
                vehicle_pos[1] + offsetY / (10 * 6378137.0)
            ]
            object_velocity = speed * 0.02
            object_heading = heading * 0.0125

            object_data = ObjectData(
                object_class=obj_class,
                object_accuracy=obj_accuracy / 100.0,
                object_position=object_position,
                object_velocity=object_velocity,
                object_heading=object_heading,
                datetime=object_datetime
            )
            object_data_list.append(object_data)

        recognition_msg = Recognition(
            vehicle_time=vehicle_time,
            vehicle_pos=vehicle_pos,
            datetime=vehicle_time,  # Assuming vehicle_time is the datetime of the message
            object_data=object_data_list
        )

        return recognition_msg



    def V2XMsgParser(self, data) :
        # Unpack the header
        fmsgHdrType = '<III'
        hdr_size = struct.calcsize(fmsgHdrType)
        hdr_data = data[:hdr_size]

        # TODO : Recognition만 처리하면 안되는구나...
        hdr, msgID, msgLen = struct.unpack(fmsgHdrType, hdr_data)
        if msgLen != len(data) - hdr_size : 
            self.get_logger().info('Invalid message length: %d' % msgLen)
            return
        
        if hdr == 0x53415445 : # received correct packet        
            if msgID == 0x016792 :
                # Unpack the recognition message
                self._proc_recognition_msg(data[hdr_size:])
            else :
                self.get_logger().info('Unknown message ID: %d' % msgID)
        else :
            self.get_logger().info('Unknown header: %d' % hdr)

        return
        
    def proc_v2x_msgs(self, r_data) :
        pass



