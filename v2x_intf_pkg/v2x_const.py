class V2XConstants:
    OBU_IP = '192.168.2.100'
    OBU_PORT = 9201
    R = 6378137.0 # Radius of the Earth in meters

    fmsgHdrType = f'<III'
    fDDateTimeType = 'HBBBBHh'  # year, month, day, hour, minute, second, microsecond
    fPosition3D = 'll'          # (latitude, longitude)
    fPositionalAccuracy = 'BBH'
    fFirstPart = f'<B {fDDateTimeType} {fPosition3D} {fPositionalAccuracy} B' # equipmentType, sDSMTimeStamp, refPos, refPosXYConf, numDetectedObjects
    fDetectedObjectCommonData = '<BBHhBhhBHBHB'
    HDR_FLAG = 0x53415445,         # hdr
    MSG_RECOGNITION = 0x16792,           # msgID for recognition

    