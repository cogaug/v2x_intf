from v2x_intf_msg.msg import Recognition, Object
import datetime
import ctypes
import socket
from v2x_intf_pkg.V2XConstants import V2XConstants as v2xconst
import v2x_intf_pkg.FmtRecognition as recogfmt
import v2x_intf_pkg.FmtCommon as fmtcommon

class MsgProcRecognition: 
  def __init__(self, logger):
    self.logger = logger

  def fromV2XMsg(self, data): # Header를 제외한 데이터를 수신받아서 Recognition 메시지로 변환
    # Create an empty v2x_recognition_msg_type instance
    recog_msg = recogfmt.v2x_recognition_msg_type()

    # Just move the header part
    hdr_size = ctypes.sizeof(fmtcommon.v2x_intf_hdr_type)
    ctypes.memmove(ctypes.addressof(recog_msg.hdr), data[:hdr_size], hdr_size)

    fixed_size = ctypes.sizeof(recogfmt.recognition_data_fixed_part_type)
    ctypes.memmove(ctypes.addressof(recog_msg.data), data[hdr_size:(hdr_size+fixed_size)], fixed_size)

    # self.logger.info(f'(V2X->) Equipment Type: {recog_msg.data.equipmentType}, refPos: {recog_msg.data.refPos.latitude}, {recog_msg.data.refPos.longitude}, refPosXYConf: {recog_msg.data.refPosXYConf.semiMajor}, {recog_msg.data.refPosXYConf.semiMinor}, {recog_msg.data.refPosXYConf.orientation} numDetectedObjects: {recog_msg.data.numDetectedObjects}')

    # Calculate the number of detected objects
    num_objects = recog_msg.data.numDetectedObjects
    # self.logger.info(f'(V2X->) Number of detected objects: {num_objects}')  

    # Calculate the size of the objects array in bytes
    objects_size = ctypes.sizeof(recogfmt.DetectedObjectCommonData) * num_objects

    # Parse the objects array
    if len(data) != hdr_size+fixed_size+objects_size:
        self.logger.error('Data is too short to include all detected objects')
        return None

    objects_array = (recogfmt.DetectedObjectCommonData * num_objects)()
    ctypes.memmove(objects_array, data[(hdr_size+fixed_size):(hdr_size+fixed_size+objects_size)], objects_size)

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
    # self.logger.info(f'(V2X->) Receive data created at {v_t}')
    vehicle_position = (
      float(recog_msg.data.refPos.latitude)/(1000.0*1000.0*10.0),  # latitude
      float(recog_msg.data.refPos.longitude)/(1000.0*1000.0*10.0)  # longitude
    )

    num_detected_objects = recog_msg.data.numDetectedObjects
    detected_objects = []
    if num_detected_objects > 255:
      # self.logger.error(f'Number of detected objects {num_detected_objects} exceeds maximum 255, set to 255')
      num_detected_objects = 255

    vehicle_id = 0
    rec_acc = 0
    for i in range(num_detected_objects):
      obj = recog_msg.objects[i]
      vehicle_id = obj.objectID >> 8
      o_t = v_t + datetime.timedelta(milliseconds=float(obj.measurementTime))
      detected_object = Object(
        detection_time = [o_t.year, o_t.month, o_t.day, o_t.hour, o_t.minute, o_t.second, o_t.microsecond],
        object_position = [float(obj.pos.offsetX)/10.0, float(obj.pos.offsetY)/10.0],
        object_velocity = float(obj.speed)*0.02,
        object_heading = float(obj.heading)*0.0125,
        object_class = obj.objType,
        recognition_accuracy = obj.objTypeCfd
      )
      rec_acc = obj.objTypeCfd  # For test missing packet
      detected_objects.append(detected_object)

      # Construct the message object
      msg = Recognition(
          vehicle_id = vehicle_id,
          vehicle_time = vehicle_time,
          vehicle_position = vehicle_position,
          object_data = detected_objects
      )
    self.logger.info(f'(->ROS2) Recognition message seq : {rec_acc}')
    return msg

  
  def toV2XMsg(self, msg):
    recog_msg = recogfmt.v2x_recognition_msg_type()

    # J3224의 sDSMTimeStamp format 구성
    recog_msg.hdr.hdr_flag = v2xconst.HDR_FLAG
    recog_msg.hdr.msgID = v2xconst.MSG_RECOGNITION

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
    recog_msg.data.refPos.latitude = int(float(msg.vehicle_position[0])*1000.0*1000.0*10.0) # Latitude in 1/10th microdegree
    recog_msg.data.refPos.longitude = int(float(msg.vehicle_position[1])*1000.0*1000.0*10.0)  # Longitude in 1/10th microdegree
    
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
      measurementTimeOffset = int((o_t-v_t).total_seconds()*1000.0) # in milliseconds for MeasurementTimeOffset type # it should have -1500 ~ 1500 in 1ms unit (-1.5 sec ~ 1.5 sec)
      if measurementTimeOffset > 1500 :
        self.logger.info(f'measurementTimeOffset is out of range {measurementTimeOffset}')
        measurementTimeOffset = 1500
      if measurementTimeOffset < -1500 : # sDSMTimeStamp보다 1.5초 빨리 디텍트한 객체
        self.logger.info(f'measurementTimeOffset is out of range {measurementTimeOffset}')
        measurementTimeOffset = -1500
        
      offsetX = int(float(obj.object_position[0])*10.0)
      offsetY = int(float(obj.object_position[1])*10.0)
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
          
      speed = int(float(obj.object_velocity) / 0.02)
      if speed > 8191 :
        self.logger.info(f'obj.object_velocity is out of range {obj.object_velocity}')
        speed = 8192 # represents "speed is unavailable"
          
      if obj.object_heading < 0.0 :
        obj.object_heading += 360.0

      heading = int((float(obj.object_heading)%360.0)/0.0125)  # in 0.0125 degree unit
      if heading > 28800 :
        heading = 28800
        self.logger.info(f'obj.object_heading is out of range {obj.object_heading}')

      objects_array[idx].objType = obj.object_class
      objects_array[idx].objTypeCfd = obj.recognition_accuracy
      objects_array[idx].objectID = (msg.vehicle_id << 8) + idx  # vehicle_id는 제어부에서 임의로 설정되는데 현재 3대의 자율차에 1,2,3으로 할당.
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
    recog_msg.hdr.msgLen = socket.htonl(fixed_part_size + objects_size)
    # self.logger.info(f'(->V2X) msg_len = {fixed_part_size + objects_size}, hdr.msgLen = {socket.ntohl(recog_msg.hdr.msgLen)}')
    hdr_bytes = ctypes.string_at(ctypes.byref(recog_msg.hdr), ctypes.sizeof(recog_msg.hdr))
    fixed_part_bytes = ctypes.string_at(ctypes.byref(recog_msg.data), ctypes.sizeof(recog_msg.data))

    objects_bytes = b''
    for idx in range(num_objects):
        object_bytes = ctypes.string_at(ctypes.byref(recog_msg.objects[idx]), ctypes.sizeof(recogfmt.DetectedObjectCommonData))
        objects_bytes += object_bytes

    recog_bytes = hdr_bytes + fixed_part_bytes + objects_bytes
    # self.logger.info(f'(->V2X) length of recog_bytes = {len(recog_bytes)}')
    return bytes(recog_bytes)

      