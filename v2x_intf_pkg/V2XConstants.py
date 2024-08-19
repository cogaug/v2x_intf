class V2XConstants:
    DEFAULT_OBU_IP = 'localhost'
    DEFAULT_OBU_PORT = 9201

    EQUIPMENT_TYPE = 2 # OBU

    R = 6378137.0 # Radius of the Earth in meters

    # HDR_FLAG = 0x53415445
    HDR_FLAG = 0x45544153 # in big-endian
    # MSG_RECOGNITION = 0x016792
    MSG_RECOGNITION = 0x92670100 # in big-endian

    