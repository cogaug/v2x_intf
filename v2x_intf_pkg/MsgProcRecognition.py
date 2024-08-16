from v2x_intf_msg.msg import Recognition, Object
import datetime
import ctypes
from v2x_intf_pkg.V2XConstants import V2XConstants as v2xconst
import v2x_intf_pkg.FmtRecognition as recogfmt
import v2x_intf_pkg.FmtCommon as fmtcommon

class MsgProcRecognition:
  """
  A class to handle Recognition messages.

  Attributes:
    logger : Logger
      The logger object to log messages.
  """  
  def __init__(self, logger):
    """
    Constructs the necessary attributes for the RecognitionMsg object.

    Args:
      logger : Logger
        The logger object to log messages.
    """
    self.logger = logger

  def fromV2XMsg(self, data): # Header를 제외한 데이터를 수신받아서 Recognition 메시지로 변환
    """
    Converts received data into a Recognition message.

    Args:
      data : bytes
        The data received, excluding the header.

    Returns:
      Recognition: The constructed Recognition message.
    """    

    # Create an empty v2x_recognition_msg_type instance
    recog_msg = recogfmt.v2x_recognition_msg_type()

    # Just move the header part
    ctypes.memmove(ctypes.addressof(recog_msg.hdr), data[:ctypes.sizeof(fmtcommon.v2x_intf_hdr_type)], ctypes.sizeof(fmtcommon.v2x_intf_hdr_type))

    # Parse the fixed part of the recognition_data_type
    offset = ctypes.sizeof(fmtcommon.v2x_intf_hdr_type)
    ctypes.memmove(ctypes.addressof(recog_msg.data), data[offset:offset + ctypes.sizeof(recogfmt.recognition_data_fixed_part_type)], ctypes.sizeof(fmtcommon.recognition_data_fixed_part_type))

    # Calculate the number of detected objects
    num_objects = recog_msg.data.numDetectedObjects

    # Calculate the size of the objects array in bytes
    objects_size = ctypes.sizeof(recogfmt.DetectedObjectCommonData) * num_objects

    # Parse the objects array
    if len(data) < offset + objects_size:
        self.logger.error('Data is too short to include all detected objects')
        return None

    objects_array = (recogfmt.DetectedObjectCommonData * num_objects)()
    ctypes.memmove(objects_array, data[offset:offset + objects_size], objects_size)

    # Assign the parsed objects array to the recognition message
    recog_msg.objects = ctypes.cast(objects_array, ctypes.POINTER(recogfmt.DetectedObjectCommonData))

    vehicle_time = [
      recog_msg.data.sDSMTimeStamp.year,
      recog_msg.data.sDSMTimeStamp.month,
      recog_msg.data.sDSMTimeStamp.day,
      recog_msg.data.sDSMTimeStamp.hour,
      recog_msg.data.sDSMTimeStamp.minute,
      recog_msg.data.sDSMTimeStamp.second // 1000,  # milliseconds to seconds
      (recog_msg.data.sDSMTimeStamp.second % 1000) * 1000 # milliseconds to microseconds
    ]

    v_t = datetime.datetime(*vehicle_time)
    self.logger.info(f'(V2X->) Receive data created at {v_t}')
    vehicle_position = (
      recog_msg.data.refPos.latitude/(1000*1000*10),  # latitude
      recog_msg.data.refPos.longitude/(1000*1000*10)  # longitude
    )

    num_detected_objects = recog_msg.data.numDetectedObjects
    detected_objects = []
    if num_detected_objects > 255:
      self.logger.error(f'Number of detected objects {num_detected_objects} exceeds maximum 255, set to 255')
      num_detected_objects = 255

    # # positional_accuracy = first_part_values[10:13]  # semiMajor, semiMinor, orientation, currently no need to use this data
    # num_detected_objects = first_part_values[13]

    # # Calculate the expected length of the detected objects part
    # detected_object_size = struct.calcsize(v2xconst.fDetectedObjectCommonData)
    # expected_detected_objects_length = num_detected_objects * detected_object_size

    # # Check if the length of detected objects part matches the expected length
    # actual_detected_objects_length = len(data) - first_part_size
    # if actual_detected_objects_length != expected_detected_objects_length:
    #   self.logger.error(f'Detected objects data length {actual_detected_objects_length} does not match expected length {expected_detected_objects_length}')
    #   return None


    # # Parse the detected objects
    # detected_objects = []
    # if num_detected_objects > 255:
    #   self.logger.error(f'Number of detected objects {num_detected_objects} exceeds maximum 255, set to 255')
    #   num_detected_objects = 255

    vehicle_id = 0
    for i in range(num_detected_objects):
    #   start_index = first_part_size + i * detected_object_size
    #   end_index = start_index + detected_object_size
    #   object_data = data[start_index:end_index]
    #   object_values = struct.unpack(v2xconst.fDetectedObjectCommonData, object_data)
      
    #   vehicle_id = object_values[2] >> 8
    #   o_t = v_t + datetime.timedelta(milliseconds=object_values[3])
    #   detected_object = Object(
    #     detection_time = [o_t.year, o_t.month, o_t.day, o_t.hour, o_t.minute, o_t.second, o_t.microsecond],
    #     object_position = [float(object_values[5])/10.0, float(object_values[6])/10.0],
    #     object_velocity = float(object_values[8])*0.02,
    #     object_heading = float(object_values[10])*0.0125,
    #     object_class = object_values[0],
    #     recognition_accuracy = object_values[1]
    #   )

    #   detected_objects.append(detected_object)

      # Construct the message object
      msg = Recognition(
          vehicle_id = vehicle_id,
          vehicle_time = vehicle_time,
          vehicle_position = vehicle_position,
          object_data = detected_objects
      )
        
    return msg

  
  def toV2XMsg(self, msg):
    """
    Converts a Recognition message into a packed data format.

    Args:
      msg : Recognition
        The Recognition message to be converted.

    Returns:
      bytes: The packed data format of the Recognition message.
    """
    recog_msg = recogfmt.v2x_recognition_msg_type()

    self.logger.info(f'(ROS->): Start to create recognition message from {msg}')


    # J3224의 sDSMTimeStamp format 구성
    recog_msg.hdr.hdr_flag = v2xconst.HDR_FLAG
    recog_msg.hdr.msgID = v2xconst.MSG_RECOGNITION
    # recog_msg.hdr.msgLen = ctypes.sizeof(recogfmt.v2x_recognition_msg_type) - ctypes.sizeof(recogfmt.v2x_intf_hdr_type)

    recog_msg.data.equipmentType = v2xconst.EQUIPMENT_TYPE
    recog_msg.data.sDSMTimeStamp.year = msg.vehicle_time[0] # year
    recog_msg.data.sDSMTimeStamp.month = msg.vehicle_time[1]  # month
    recog_msg.data.sDSMTimeStamp.day = msg.vehicle_time[2]  # day
    recog_msg.data.sDSMTimeStamp.hour = msg.vehicle_time[3] # hour
    recog_msg.data.sDSMTimeStamp.minute = msg.vehicle_time[4] # minute
    recog_msg.data.sDSMTimeStamp.second = msg.vehicle_time[5]*1000+(msg.vehicle_time[6]//1000) # milliseconds
    recog_msg.data.sDSMTimeStamp.offset = 9*60 # Timezone in minutes

    # Create datetime objects, including milliseconds to calculate measurementTimeOffset
    v_t = datetime.datetime(
      msg.vehicle_time[0],  # year
      msg.vehicle_time[1],  # month
      msg.vehicle_time[2],  # day
      msg.vehicle_time[3],  # hour
      msg.vehicle_time[4],  # minute
      msg.vehicle_time[5],  # second
      msg.vehicle_time[6]   # microsecond
    )
    self.logger.info(f'(ROS->): Data created at {v_t}')

    recog_msg.data.refPos.latitude = int(msg.vehicle_position[0]*1000*1000*10) # Latitude in 1/10th microdegree
    recog_msg.data.refPos.longitude = int(msg.vehicle_position[1]*1000*1000*10)  # Longitude in 1/10th microdegree
    

    if recog_msg.data.refPos.latitude > 900000000 or recog_msg.data.refPos.latitude < -900000000 :
      self.logger.info(f'Latitude is out of range {recog_msg.data.refPos.latitude}')
      return None
      
    if recog_msg.data.refPos.longitude > 1800000000 or recog_msg.data.refPos.longitude < -1800000000 :
      self.logger.info(f'Longitude is out of range {recog_msg.data.refPos.longitude}')
      return None
    
    recog_msg.data.refPosXYConf.semiMajor = 255  # semiMajor
    recog_msg.data.refPosXYConf.semiMinor = 255  # semiMinor
    recog_msg.data.refPosXYConf.orientation = 65535 # orientation
    num_objects = len(msg.object_data)
    if num_objects <= 256 :
      recog_msg.data.numDetectedObjects = num_objects
    else :
      recog_msg.data.numDetectedObjects = 255
      num_objects = 255
      self.logger.info(f'Number of detected objects is over 256, set to 255')

    objects_array = (recogfmt.DetectedObjectCommonData * num_objects)()
    for idx, obj in enumerate(msg.object_data) :
      if idx >= num_objects :
        break

      # Create datetime objects, including milliseconds to calculate measurementTimeOffset
      o_t = datetime.datetime(
        obj.detection_time[0],  # year
        obj.detection_time[1],  # month
        obj.detection_time[2],  # day
        obj.detection_time[3],  # hour
        obj.detection_time[4],  # minute
        obj.detection_time[5],  # second
        obj.detection_time[6]   # microsecond
      )
      measurementTimeOffset = int((o_t-v_t).total_seconds()*1000) # in milliseconds for MeasurementTimeOffset type # it should have -1500 ~ 1500 in 1ms unit (-1.5 sec ~ 1.5 sec)
      if measurementTimeOffset > 1500 :
        self.logger.info(f'measurementTimeOffset is out of range {measurementTimeOffset}')
        measurementTimeOffset = 1500
      if measurementTimeOffset < -1500 : # sDSMTimeStamp보다 1.5초 빨리 디텍트한 객체
        self.logger.info(f'measurementTimeOffset is out of range {measurementTimeOffset}')
        measurementTimeOffset = -1500
        
      offsetX = int(obj.object_position[0]*10)
      offsetY = int(obj.object_position[1]*10)
      if offsetX > 32767 :
        self.logger.info(f'obj.object_position is out of range {offsetX}')
        offsetX = 32767
      if offsetX < -32767 :
        self.logger.info(f'obj.object_position is out of range {offsetX}')
        offsetX = -32767
      if offsetY > 32767 :
        self.logger.info(f'obj.object_position is out of range {offsetY}')
        offsetY = 32767
      if offsetY < -32767 :
        self.logger.info(f'obj.object_position is out of range {offsetY}')
        offsetY = -32767
          
      speed = int(obj.object_velocity / 0.02)
      if speed > 8191 :
        self.logger.info(f'obj.object_velocity is out of range {obj.object_velocity}')
        speed = 8192 # represents "speed is unavailable"
          
      if obj.object_heading < 0.0 :
        obj.object_heading += 360.0

      heading = int(((obj.object_heading)%360.0)/0.0125)  # in 0.0125 degree unit
      if heading > 28800 :
        heading = 28800
        self.logger.info(f'obj.object_heading is out of range {obj.object_heading}')

      objects_array[idx].objType = obj.object_class
      objects_array[idx].objTypeCfd = obj.recognition_accuracy
      objects_array[idx].objectID = msg.vehicle_id << 8 + idx  # vehicle_id는 제어부에서 임의로 설정되는데 현재 3대의 자율차에 1,2,3으로 할당.
      objects_array[idx].measurementTime = measurementTimeOffset
      objects_array[idx].timeConfidence = 0
      objects_array[idx].pos.offsetX = offsetX
      objects_array[idx].pos.offsetY = offsetY
      objects_array[idx].posConfidence = 0
      objects_array[idx].speed = speed
      objects_array[idx].speedConfidence = 0
      objects_array[idx].heading = heading
      objects_array[idx].headingConf = 0

    recog_msg.objects = ctypes.cast(objects_array, ctypes.POINTER(recogfmt.DetectedObjectCommonData))

    fixed_part_size = ctypes.sizeof(recogfmt.recognition_data_fixed_part_type)
    objects_size = num_objects * ctypes.sizeof(recogfmt.DetectedObjectCommonData)

    self.logger.info(f'(ROS->): hdr size {ctypes.sizeof(fmtcommon.v2x_intf_hdr_type)}, fixed part size {fixed_part_size}, objects size {objects_size}')
    recog_msg.hdr.msgLen = fixed_part_size + objects_size
    self.logger.info(f'(ROS->): msgLen {recog_msg.hdr.msgLen}')
    self.logger.info(f'(ROS->): recognition message created with lenght {len(bytes(recog_msg))}')
    return bytes(recog_msg)

      



    # packed_objects = b''
    # num_object = 0
    # for idx, obj in enumerate(msg.object_data) :
    #   # Create datetime objects, including milliseconds to calculate measurementTimeOffset
    #   o_t = datetime.datetime(
    #     obj.detection_time[0],  # year
    #     obj.detection_time[1],  # month
    #     obj.detection_time[2],  # day
    #     obj.detection_time[3],  # hour
    #     obj.detection_time[4],  # minute
    #     obj.detection_time[5],  # second
    #     obj.detection_time[6]   # microsecond
    #   )
    #   measurementTimeOffset = int((o_t-v_t).total_seconds()*1000) # in milliseconds for MeasurementTimeOffset type # it should have -1500 ~ 1500 in 1ms unit (-1.5 sec ~ 1.5 sec)
          
    #   if measurementTimeOffset > 1500 or measurementTimeOffset < -1500 : # sDSMTimeStamp보다 1.5초 빨리 디텍트한 객체
    #     self.logger.info(f'measurementTimeOffset is out of range {measurementTimeOffset}')
    #     continue
          
    #   object_id = msg.vehicle_id << 8 + idx  # vehicle_id는 제어부에서 임의로 설정되는데 현재 3대의 자율차에 1,2,3으로 할당.
          
    #   offsetX = int(obj.object_position[0]*10)
    #   offsetY = int(obj.object_position[1]*10)
    #   if offsetX > 32767 or offsetX < -32767 or offsetY > 32767 or offsetY < -32767 :
    #     self.logger.info(f'obj.object_position is out of range {obj.object_position}')
    #     continue
          
    #   speed = int(obj.object_velocity / 0.02)
    #   if speed > 8191 :
    #     self.logger.info(f'obj.object_velocity is out of range {obj.object_velocity}')
    #     speed = 8192 # represents "speed is unavailable"
          
    #   if obj.object_heading < 0.0 :
    #     obj.object_heading += 360.0

    #   heading = int(((obj.object_heading)%360.0)/0.0125)  # in 0.0125 degree unit
    #   if heading > 28800 :
    #     self.logger.info(f'obj.object_heading is out of range {obj.object_heading}')
    #     continue
                    
    #   packed_object = struct.pack(
    #     v2xconst.fDetectedObjectCommonData,
    #     obj.object_class,
    #     obj.recognition_accuracy,
    #     object_id,
    #     measurementTimeOffset,
    #     0, # timeConfidence
    #     offsetX, offsetY,
    #     0, # posConfidenceSet
    #     speed,
    #     0, # speedConfidence
    #     heading,
    #     0 # headingConfidence
    #   )
    #   packed_objects += packed_object
    #   num_object += 1
    #   if num_object >= 256 :
    #     break


    # first_part = struct.pack(
    #   v2xconst.fFirstPart,
    #   v2xconst.EQUIPMENT_TYPE,
    #   *sDSMTimeStamp,
    #   *position3D,
    #   *positionAccuracy,
    #   num_object
    # )

    # packed_data = first_part + packed_objects

    # # Convert header to C struct data type
    # # Ref : v2x_intf_hdr_type
    # hdr_data = struct.pack(
    #   v2xconst.fmsgHdrType,
    #   v2xconst.HDR_FLAG,          # hdr
    #   v2xconst.MSG_RECOGNITION,   # msgID for recognition
    #   len(packed_data)            # msgLen
    # )

    # return hdr_data + packed_data
