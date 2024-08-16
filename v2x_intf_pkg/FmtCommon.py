import ctypes

# Define the v2x_intf_hdr_type structure
class v2x_intf_hdr_type(ctypes.Structure):
    _fields_ = [
        ("hdr_flag", ctypes.c_uint),  # unsigned int
        ("msgID", ctypes.c_uint),     # unsigned int
        ("msgLen", ctypes.c_uint)     # unsigned int
    ]

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
