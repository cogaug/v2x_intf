
typedef struct {
  unsigned short year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  unsigned short second;
  short offset;
} DDateTimeType;


typedef struct {
  long latitude;
  long longitude;
} Position3D;

typedef struct {
  unsigned char semiMajor;
  unsigned char semiMinor;
  unsigned short orientation;
} PositionalAccuracy;

typedef struct {
  short offsetX;
  short offsetY;
} PositionOffsetXYZ;

typedef struct {
  unsigned char objType;
  unsigned char objTypeCfd;
  unsigned short objectID;
  short measurementTime;
  unsigned char timeConfidence;
  PositionOffsetXYZ pos;
  unsigned char posConfidence;
  unsigned short speed;
  unsigned char speedConfidence;
  unsigned short heading;
  unsigned char headingConf;
} DetectedObjectCommonData;

#define NUM_OF_OBJECTS 255
typedef struct {
  unsigned char equipmentType;
  DDateTimeType sDSMTimeStamp;
  Position3D  refPos;
  PositionalAccuracy refPosXYConf;
  unsigned char numDetectedObjects;
  DetectedObjectCommonData objects[NUM_OF_OBJECTS];
} v2x_intf_type;