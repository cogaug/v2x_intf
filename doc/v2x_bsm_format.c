// Follow ITSK-00130

#define MSGID 20  // BSM  DSRCmsgID

// 3.3. CDF_PositionalAccuracy
typedef struct {
  unsigned char semiMajor;    // DE_SemiMajorAxisAccuracy
  unsigned char semiMinor;    // DE_SemiMinorAxisAccuracy
  unsigned short orientation; // DE_SemiMajorAxisOrientation
} CDF_PositionalAccuracy;     // DF_PositionalAccuracy

// 3.4. CDF_AccelerationSet4Way
typedef struct {
    short lon; // DE_Acceleration
    short lat; // DE_Acceleration
    char vert; // DE_VerticalAcceleration
    short yaw; // DE_YawRate
} CDF_AccelerationSet4Way; // DF_AccelerationSet4Way

// 3.5. CDF_BrakeSystemStatus
typedef struct {
    unsigned char wheelBrakes; // DE_BrakeAppliedStatus
    unsigned char traction; // DE_TractionControlStatus
    unsigned char abs; // DE_AntiLockBrakeStatus
    unsigned char scs; // DE_StabilityControlStatus
    unsigned char brakeBoost; // DE_BrakeBoostApplied
    unsigned char auxBrakes; // DE_AuxiliaryBrakeStatus
} CDF_BrakeSystemStatus; // DF_BrakeSystemStatus

// 3.6. CDF_VehicleSize
typedef struct {
    unsigned short width; // DE_VehicleWidth
    unsigned short length; // DE_VehicleLength
} CDF_VehicleSize; // DF_VehicleSize


// 3.7. CDF_VehicleSafetyExtensions

// 3.8. CDF_PathHistory


// 3.9. CDF_PathHistoryPointList

// 3.10.CDF_PathHistoryPoint
#define CDF_PHPOINT_OPTION_SPEED        0x00
#define CDF_PHPOINT_OPTION_POSACCURACY  0x01
#define CDF_PHPOINT_OPTION_HEADING      0x02
typedef struct {
    unsigned char option;
    int latOffset; // DE_OffsetLL_B18
    int lonOffset; // DE_OffsetLL_B18
    short elevationOffset; // DE_VertOffset-B12
    unsigned short timeOffset; // DE_TimeOffset
    union {  // Option dependent
        unsigned short speed; // DE_Speed
        CDF_PositionalAccuracy posAccuracy; // DF_PositionalAccuracy
        unsigned char heading; // DE_CoarseHeading
    } u;
} CDF_PathHistoryPoint; // CDF_PathHistoryPoint



// 3.11.CDF_PathPrediction
typedef struct {
    short radiusOfCurve;    // DE_RadiusOfCurvature
    unsigned char confidence; // DE_Confidence
} CDF_PathPrediction; // DF_PathPrediction

// 3.12.CDF_SpecialVehicleExtensions


// 3.13.CDF_EventDescription