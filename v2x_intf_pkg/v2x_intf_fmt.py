import ctypes

# Define the DDateTimeType structure
class DDateTimeType(ctypes.Structure):
    _fields_ = [
        ("year", ctypes.c_ushort),   # unsigned short
        ("month", ctypes.c_ubyte),   # unsigned char
        ("day", ctypes.c_ubyte),     # unsigned char
        ("hour", ctypes.c_ubyte),    # unsigned char
        ("minute", ctypes.c_ubyte),  # unsigned char
        ("second", ctypes.c_ushort), # unsigned short
        ("offset", ctypes.c_short)   # short
    ]

# Define the Position3D structure
class Position3D(ctypes.Structure):
    _fields_ = [
        ("latitude", ctypes.c_long),   # long
        ("longitude", ctypes.c_long)   # long
    ]

# Define the PositionalAccuracy structure
class PositionalAccuracy(ctypes.Structure):
    _fields_ = [
        ("semiMajor", ctypes.c_ubyte),    # unsigned char
        ("semiMinor", ctypes.c_ubyte),    # unsigned char
        ("orientation", ctypes.c_ushort)  # unsigned short
    ]

# Define the PositionOffsetXYZ structure
class PositionOffsetXYZ(ctypes.Structure):
    _fields_ = [
        ("offsetX", ctypes.c_short),  # short
        ("offsetY", ctypes.c_short)   # short
    ]

# Define the DetectedObjectCommonData structure
class DetectedObjectCommonData(ctypes.Structure):
    _fields_ = [
        ("objType", ctypes.c_ubyte),         # unsigned char
        ("objTypeCfd", ctypes.c_ubyte),      # unsigned char
        ("objectID", ctypes.c_ushort),       # unsigned short
        ("measurementTime", ctypes.c_short), # short
        ("timeConfidence", ctypes.c_ubyte),  # unsigned char
        ("pos", PositionOffsetXYZ),          # PositionOffsetXYZ structure
        ("posConfidence", ctypes.c_ubyte),   # unsigned char
        ("speed", ctypes.c_ushort),          # unsigned short
        ("speedConfidence", ctypes.c_ubyte), # unsigned char
        ("heading", ctypes.c_ushort),        # unsigned short
        ("headingConf", ctypes.c_ubyte)      # unsigned char
    ]

# Define the v2x_intf_hdr_type structure
class v2x_intf_hdr_type(ctypes.Structure):
    _fields_ = [
        ("hdr_flag", ctypes.c_uint),  # unsigned int
        ("msgID", ctypes.c_uint),     # unsigned int
        ("msgLen", ctypes.c_uint)     # unsigned int
    ]

# Define the recognition_data_type structure
class recognition_data_type(ctypes.Structure):
    _fields_ = [
        ("equipmentType", ctypes.c_ubyte),    # unsigned char
        ("sDSMTimeStamp", DDateTimeType),     # DDateTimeType structure
        ("refPos", Position3D),               # Position3D structure
        ("refPosXYConf", PositionalAccuracy), # PositionalAccuracy structure
        ("numDetectedObjects", ctypes.c_ubyte), # unsigned char
        ("objects", ctypes.POINTER(DetectedObjectCommonData)) # Pointer to DetectedObjectCommonData array
    ]

class recognition_data_fixed_part_type(ctypes.Structure):
    _fields_ = [
        ("equipmentType", ctypes.c_ubyte),    # unsigned char
        ("sDSMTimeStamp", DDateTimeType),     # DDateTimeType structure
        ("refPos", Position3D),               # Position3D structure
        ("refPosXYConf", PositionalAccuracy), # PositionalAccuracy structure
        ("numDetectedObjects", ctypes.c_ubyte) # unsigned char
    ]

# Define the v2x_recognition_msg_type structure
class v2x_recognition_msg_type(ctypes.Structure):
    _fields_ = [
        ("hdr", v2x_intf_hdr_type),     # v2x_intf_hdr_type structure
        ("data", recognition_data_fixed_part_type) # recognition_data_fixed_part_type structure
        ("objects", ctypes.POINTER(DetectedObjectCommonData)) # Pointer to DetectedObjectCommonData array
    ]

# # Example usage:
# def create_and_send_recognition_message(num_objects):
#     # Create an instance of v2x_recognition_msg_type
#     recog_msg = v2x_recognition_msg_type()

#     # Populate the hdr part
#     recog_msg.hdr.hdr_flag = HDR_FLAG
#     recog_msg.hdr.msgID = RECOGNITION_MSG
#     recog_msg.hdr.msgLen = ctypes.sizeof(v2x_recognition_msg_type) - ctypes.sizeof(v2x_intf_hdr_type)

#     # Populate the recognition_data_type part
#     recog_msg.data.equipmentType = 1
#     recog_msg.data.sDSMTimeStamp.year = 2024
#     recog_msg.data.sDSMTimeStamp.month = 8
#     recog_msg.data.sDSMTimeStamp.day = 12
#     recog_msg.data.sDSMTimeStamp.hour = 10
#     recog_msg.data.sDSMTimeStamp.minute = 30
#     recog_msg.data.sDSMTimeStamp.second = 500
#     recog_msg.data.sDSMTimeStamp.offset = 0
#     recog_msg.data.refPos.latitude = 123456789
#     recog_msg.data.refPos.longitude = 987654321
#     recog_msg.data.refPosXYConf.semiMajor = 50
#     recog_msg.data.refPosXYConf.semiMinor = 40
#     recog_msg.data.refPosXYConf.orientation = 200
#     recog_msg.data.numDetectedObjects = num_objects

#     # Create the objects array based on numDetectedObjects
#     objects_array = (DetectedObjectCommonData * num_objects)()
#     for i in range(num_objects):
#         objects_array[i].objType = i + 1
#         objects_array[i].objTypeCfd = (i + 1) % 256
#         objects_array[i].objectID = 1234 + i
#         objects_array[i].measurementTime = 100 + i
#         objects_array[i].timeConfidence = 80 + i % 10
#         objects_array[i].pos.offsetX = 10 + i
#         objects_array[i].pos.offsetY = 20 + i
#         objects_array[i].posConfidence = 60 + i % 5
#         objects_array[i].speed = 500 + i
#         objects_array[i].speedConfidence = 70 + i % 7
#         objects_array[i].heading = 180 + i
#         objects_array[i].headingConf = 90 + i % 10

#     # Assign the objects array to the recognition_data_type
#     recog_msg.data.objects = ctypes.cast(objects_array, ctypes.POINTER(DetectedObjectCommonData))

#     # Convert the structure to bytes
#     data = bytes(recog_msg)

#     # Send the data over a socket or other method
#     sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     sock.connect(("localhost", 8080))  # Replace with the appropriate address and port
#     sock.sendall(data)
#     sock.close()

# # Call the function to create and send the recognition message with the desired number of objects
# create_and_send_recognition_message(10)
