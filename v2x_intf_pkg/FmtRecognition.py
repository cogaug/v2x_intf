import ctypes
import v2x_intf_pkg.FmtCommon as fmtcommon

# Define the DetectedObjectCommonData structure
class DetectedObjectCommonData(ctypes.Structure):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("objType", ctypes.c_ubyte),            # 1 unsigned char
        ("objTypeCfd", ctypes.c_ubyte),         # 1 unsigned char
        ("objectID", ctypes.c_ushort),          # 2 unsigned short
        ("measurementTime", ctypes.c_short),    # 2 short
        ("timeConfidence", ctypes.c_ubyte),     # 1 unsigned char
        ("pos", fmtcommon.PositionOffsetXYZ),   # 4 PositionOffsetXYZ structure
        ("posConfidence", ctypes.c_ubyte),      # 1 unsigned char
        ("speed", ctypes.c_ushort),             # 2 unsigned short
        ("speedConfidence", ctypes.c_ubyte),    # 1 unsigned char
        ("heading", ctypes.c_ushort),           # 2 unsigned short
        ("headingConf", ctypes.c_ubyte)         # 1 unsigned char
    ]

# # Define the recognition_data_type structure
# class recognition_data_type(ctypes.Structure):
#     _fields_ = [
#         ("equipmentType", ctypes.c_ubyte),    # unsigned char
#         ("sDSMTimeStamp", fmtcommon.DDateTimeType),     # DDateTimeType structure
#         ("refPos", fmtcommon.Position3D),               # Position3D structure
#         ("refPosXYConf", fmtcommon.PositionalAccuracy), # PositionalAccuracy structure
#         ("numDetectedObjects", ctypes.c_ubyte), # unsigned char
#         ("objects", ctypes.POINTER(DetectedObjectCommonData)) # Pointer to DetectedObjectCommonData array
#     ]

class recognition_data_fixed_part_type(ctypes.Structure):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("equipmentType", ctypes.c_ubyte),    # unsigned char
        ("sDSMTimeStamp", fmtcommon.DDateTimeType),     # DDateTimeType structure
        ("refPos", fmtcommon.Position3D),               # Position3D structure
        ("refPosXYConf", fmtcommon.PositionalAccuracy), # PositionalAccuracy structure
        ("numDetectedObjects", ctypes.c_ubyte) # unsigned char
    ]

# Define the v2x_recognition_msg_type structure
class v2x_recognition_msg_type(ctypes.Structure):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("hdr", fmtcommon.v2x_intf_hdr_type),     # v2x_intf_hdr_type structure
        ("data", recognition_data_fixed_part_type), # recognition_data_fixed_part_type structure
        ("objects", ctypes.POINTER(DetectedObjectCommonData)) # Pointer to DetectedObjectCommonData array
    ]
