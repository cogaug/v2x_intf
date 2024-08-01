from v2x_intf_msg.msg import Recognition, Object
import struct
import datetime
from v2x_intf_pkg.v2x_const import V2XConstants as v2xconst


class Parser :
  """
  A class to parse incoming data into ROS2 messages according to the msg_type field.

  Attributes:
    logger : Logger
      The logger object to log messages.
  """  
  def __init__(self, logger):
    """
    Constructs the necessary attributes for the Parser object.

    Args:
      logger : Logger
        The logger object to log messages.
    """
    self.logger = logger

  def parse(self, pkd_data):
    """
    Parses the packed data into a Recognition message.

    Args:
      pkd_data : bytes
        The packed data to be parsed.

    Returns:
      Recognition: The parsed Recognition message or None if parsing fails.
    """
    # Ensure pkd_data is bytes
    if not isinstance(pkd_data, bytes):
      self.logger.error("Input data is not bytes")
      return None

    # Unpack the header
    header_size = struct.calcsize(v2xconst.fmsgHdrType)
    hdr_data = pkd_data[:header_size]
    hdr_values = struct.unpack(v2xconst.fmsgHdrType, hdr_data)

    hdr_flag = hdr_values[0]
    msg_type = hdr_values[1]
    msg_len = hdr_values[2]
    
    self.logger.info(f'Header data: {hdr_data}')
    self.logger.info(f'--> hdr_flag: {hdr_flag:#X}, msg_type: {msg_type:#X}, msg_len: {msg_len}')

    if hdr_flag is v2xconst.HDR_FLAG:
      self.info('Invalid header flag: %d' % hdr_flag)
      return None
    else :
      if len(pkd_data) - header_size != msg_len:  # Check message length
        self.logger.error(f"Data size {len(pkd_data) - header_size} does not match header msg_len {msg_len}")
        return None
      if msg_type == v2xconst.MSG_RECOGNITION :
        return RecognitionMsg(self.logger).fromV2XMsg(pkd_data[header_size:])
      else :
        self.logger.info(f'Unknown message type: {msg_type} != {v2xconst.MSG_RECOGNITION}')
        return None
    

class RecognitionMsg :
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
    first_part_size = struct.calcsize(v2xconst.fFirstPart)
    first_part_data = data[:first_part_size]
    first_part_values = struct.unpack(v2xconst.fFirstPart, first_part_data)
    equipment_type = first_part_values[0]

    self.logger.info(f"V2X Recognition message from {'OBU' if equipment_type == 2 else 'RSU'}")

    vehicle_time = first_part_values[1:8]  # year, month, day, hour, minute, milliseconds, timeoffset
    self.logger.info(f'vehicle_time: {vehicle_time}')
    vehicle_time = vehicle_time[:5] + (vehicle_time[5] // 1000, (vehicle_time[5] % 1000)*1000,)  # milliseconds to seconds and microseconds
    self.logger.info(f'---> vehicle_time: {vehicle_time}')
    v_t = datetime.datetime(*vehicle_time)
    self.logger.info(f'---> v_t: {v_t}')

    self.logger.info(f'vehicle_position: {first_part_values[8:10]}')
    vehicle_position = (
      first_part_values[8]/(1000*1000*10),  # latitude
      first_part_values[9]/(1000*1000*10)  # longitude
    )
    self.logger.info(f'---> vehicle_position: {vehicle_position}')

    # positional_accuracy = first_part_values[10:13]  # semiMajor, semiMinor, orientation, currently no need to use this data
    num_detected_objects = first_part_values[13]

    self.logger.info(f'--> equipment_type: {equipment_type}, vehicle_time: {vehicle_time}, vehicle_position: {vehicle_position}, num_detected_objects: {num_detected_objects}')


    # Calculate the expected length of the detected objects part
    detected_object_size = struct.calcsize(v2xconst.fDetectedObjectCommonData)
    expected_detected_objects_length = num_detected_objects * detected_object_size

    # Check if the length of detected objects part matches the expected length
    actual_detected_objects_length = len(data) - first_part_size
    if actual_detected_objects_length != expected_detected_objects_length:
      self.logger.error(f'Detected objects data length {actual_detected_objects_length} does not match expected length {expected_detected_objects_length}')
      return None


    # Parse the detected objects
    detected_objects = []
    if num_detected_objects > 256:
      self.logger.error(f'Number of detected objects {num_detected_objects} exceeds maximum 256, set to 256')
      num_detected_objects = 256

    vehicle_id = 0
    for i in range(num_detected_objects):
      start_index = first_part_size + i * detected_object_size
      end_index = start_index + detected_object_size
      object_data = data[start_index:end_index]
      object_values = struct.unpack(v2xconst.fDetectedObjectCommonData, object_data)
      
      vehicle_id = object_values[2] >> 8
      self.logger.info(f'measurementTimeOffset: {object_values[3]}')
      o_t = v_t + datetime.timedelta(milliseconds=object_values[3])
      self.logger.info(f'detection_time: {o_t}')
      detected_object = Object(
        detection_time = [o_t.year, o_t.month, o_t.day, o_t.hour, o_t.minute, o_t.second, o_t.microsecond],
        object_position = [float(object_values[5])/10.0, float(object_values[6])/10.0],
        object_velocity = float(object_values[8])*0.02,
        object_heading = float(object_values[10])*0.0125,
        object_class = object_values[0],
        recognition_accuracy = object_values[1]
      )

      detected_objects.append(detected_object)
      self.logger.info(f'Detected object {i}: {detected_object}')

      # Construct the message object
      msg = Recognition(
        vehicle_id = vehicle_id,
        vehicle_time = vehicle_time,
        vehicle_position = vehicle_position,
        object_data = detected_objects
      )
      
      self.logger.info(f'Constructed Recognition message: {msg}')
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
    self.logger.info(f'msg.vehicle_time: {msg.vehicle_time}')
    self.logger.info(f'--> sDSMTimeStamp: {sDSMTimeStamp}')
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
    position3D = (
      int(msg.vehicle_position[0]*1000*1000*10), # Latitude in 1/10th microdegree
      int(msg.vehicle_position[1]*1000*1000*10)  # Longitude in 1/10th microdegree
    )
    self.logger.info(f'msg.vehicle_position: {msg.vehicle_position}')
    self.logger.info(f'--> position3D: {position3D}')
    if position3D[0] > 900000000 or position3D[0] < -900000000 :
      self.logger.info(f'--> Latitude is out of range')
      return None
      
    if position3D[1] > 1800000000 or position3D[1] < -1800000000 :
      self.logger.info(f'--> Longitude is out of range')
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
      o_t = datetime.datetime(
        obj.detection_time[0],  # year
        obj.detection_time[1],  # month
        obj.detection_time[2],  # day
        obj.detection_time[3],  # hour
        obj.detection_time[4],  # minute
        obj.detection_time[5],  # second
        obj.detection_time[6]   # microsecond
      )
      self.logger.info(f'obj.detection_time: {obj.detection_time}')
      measurementTimeOffset = int((o_t-v_t).total_seconds()*1000) # in milliseconds for MeasurementTimeOffset type # it should have -1500 ~ 1500 in 1ms unit (-1.5 sec ~ 1.5 sec)
      self.logger.info(f'--> (dt2-dt1).total_seconds(): {(o_t-v_t).total_seconds()}')
      self.logger.info(f'--> measurementTimeOffset: {measurementTimeOffset}')
          
      if measurementTimeOffset > 1500 or measurementTimeOffset < -1500 : # sDSMTimeStamp보다 1.5초 빨리 디텍트한 객체
        self.logger.info(f'--> measurementTimeOffset is out of range')
        continue
          
      object_id = msg.vehicle_id << 8 + idx  # vehicle_id는 제어부에서 임의로 설정되는데 현재 3대의 자율차에 1,2,3으로 할당.
      self.logger.info(f'--> object_id: {object_id:#X}')
          
      offsetX = int(obj.object_position[0]*10)
      offsetY = int(obj.object_position[1]*10)
      self.logger.info(f'obj.object_position {obj.object_position}')
      self.logger.info(f'--> offsetX: {offsetX}, offsetY: {offsetY}')
      if offsetX > 32767 or offsetX < -32767 or offsetY > 32767 or offsetY < -32767 :
        self.logger.info(f'--> offsetX or offsetY is out of range')
        continue
          
      speed = int(obj.object_velocity / 0.02)
      self.logger.info(f'obj.object_velocity: {obj.object_velocity}')
      self.logger.info(f'--> speed: {speed}')
      if speed > 8191 :
        self.logger.info(f'--> speed is out of range')
        speed = 8192 # represents "speed is unavailable"
          
      if obj.object_heading < 0.0 :
        obj.object_heading += 360.0

      heading = int(((obj.object_heading)%360.0)/0.0125)  # in 0.0125 degree unit
      self.logger.info(f'obj.object_heading: {obj.object_heading}')
      self.logger.info(f'--> heading: {heading}')
      if heading > 28800 :
        self.logger.info(f'--> heading is out of range')
        continue
                    
      packed_object = struct.pack(
        v2xconst.fDetectedObjectCommonData,
        obj.object_class,
        obj.recognition_accuracy,
        object_id,
        measurementTimeOffset,
        0, # timeConfidence
        offsetX, offsetY,
        0, # posConfidenceSet
        speed,
        0, # speedConfidence
        heading,
        0 # headingConfidence
      )
      packed_objects += packed_object
      num_object += 1
      if num_object >= 256 :
        self.logger.info(f'--> Support 256 objects')
        break


    first_part = struct.pack(
      v2xconst.fFirstPart,
      v2xconst.EQUIPMENT_TYPE,
      *sDSMTimeStamp,
      *position3D,
      *positionAccuracy,
      num_object
    )

    packed_data = first_part + packed_objects

    # Convert header to C struct data type
    # Ref : v2x_intf_hdr_type
    hdr_data = struct.pack(
      v2xconst.fmsgHdrType,
      v2xconst.HDR_FLAG,          # hdr
      v2xconst.MSG_RECOGNITION,   # msgID for recognition
      len(packed_data)            # msgLen
    )

    return hdr_data + packed_data
