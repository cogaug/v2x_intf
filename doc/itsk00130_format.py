import ctypes

# Define the CDF_PositionalAccuracy structure
class CDF_PositionalAccuracy(ctypes.Structure):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("semiMajor", ctypes.c_ubyte),      # unsigned char
        ("semiMinor", ctypes.c_ubyte),      # unsigned char
        ("orientation", ctypes.c_ushort)    # unsigned short
    ]

# Define the CDF_AccelerationSet4Way structure
class CDF_AccelerationSet4Way(ctypes.Structure):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("lon", ctypes.c_short),            # short
        ("lat", ctypes.c_short),            # short
        ("vert", ctypes.c_char),            # char
        ("yaw", ctypes.c_short)             # short
    ]

# Define the CDF_BrakeSystemStatus structure
class CDF_BrakeSystemStatus(ctypes.Structure):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("wheelBrakes", ctypes.c_ubyte),    # unsigned char
        ("traction", ctypes.c_ubyte),       # unsigned char
        ("abs", ctypes.c_ubyte),            # unsigned char
        ("scs", ctypes.c_ubyte),            # unsigned char
        ("brakeBoost", ctypes.c_ubyte),     # unsigned char
        ("auxBrakes", ctypes.c_ubyte)       # unsigned char
    ]

# Define the CDF_VehicleSize structure
class CDF_VehicleSize(ctypes.Structure):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("width", ctypes.c_ushort),         # unsigned short
        ("length", ctypes.c_ushort)         # unsigned short
    ]

# Define the union inside CDF_PathHistoryPoint
class CDF_Union(ctypes.Union):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("speed", ctypes.c_ushort),         # unsigned short
        ("posAccuracy", CDF_PositionalAccuracy),  # CDF_PositionalAccuracy
        ("heading", ctypes.c_ubyte)         # unsigned char
    ]

# Define the CDF_PathHistoryPoint structure
class CDF_PathHistoryPoint(ctypes.Structure):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("option", ctypes.c_ubyte),         # unsigned char
        ("latOffset", ctypes.c_int),        # int
        ("lonOffset", ctypes.c_int),        # int
        ("elevationOffset", ctypes.c_short), # short
        ("timeOffset", ctypes.c_ushort),    # unsigned short
        ("u", CDF_Union)                    # union
    ]

# Define the CDF_PathPrediction structure
class CDF_PathPrediction(ctypes.Structure):
    _pack_ = 1  # Force packing to eliminate padding
    _fields_ = [
        ("radiusOfCurve", ctypes.c_short),  # short
        ("confidence", ctypes.c_ubyte)      # unsigned char
    ]
