class V2XConstants:
    OBU_IP = '192.168.2.100'
    OBU_PORT = 9201

    fmsgHdrType = '<III'
    fDDateTimeType = 'HBBBBHh'  # year, month, day, hour, minute, second, microsecond
    fPosition3D = 'll'          # (latitude, longitude)
    fPositionalAccuracy = 'BBH'
    fFirstPart = f'<B {fDDateTimeType} {fPosition3D} {fPositionalAccuracy} B' # equipmentType, sDSMTimeStamp, refPos, refPosXYConf, numDetectedObjects
    fDetectedObjectCommonData = '<BBHhBhhBHBHB'
    HDR_FLAG = 0x53415445,         # hdr
    MSG_RECOGNITION = 0x016792,           # msgID for recognition

    