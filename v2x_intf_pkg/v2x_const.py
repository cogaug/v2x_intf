class V2XConstants:
    OBU_IP = 'localhost'
    OBU_PORT = 9201

    EQUIPMENT_TYPE = 2 # OBU

    R = 6378137.0 # Radius of the Earth in meters

    fmsgHdrType = '<III'
    fDDateTimeType = 'HBBBBHh'  # year, month, day, hour, minute, second, microsecond
    fPosition3D = 'll'          # (latitude, longitude)
    fPositionalAccuracy = 'BBH'
    fFirstPart = f'<B {fDDateTimeType} {fPosition3D} {fPositionalAccuracy} B' # equipmentType, sDSMTimeStamp, refPos, refPosXYConf, numDetectedObjects
    fDetectedObjectCommonData = '<BBHhBhhBHBHB'
    HDR_FLAG = 0x53415445
    MSG_RECOGNITION = 0x016792

    