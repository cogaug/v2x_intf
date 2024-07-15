
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


#define HDR             0x53415445
#define RECOGNITION_MSG 0x016792
#define SITUATION_MSG   0x016793
#define NEGOTIATION_MSG 0x016794

typedef struct {
  unsigned int hdr;
  unsigned int msgID;
  unsigned int msgLen;
} v2x_intf_hdr_type;

#define NUM_OF_OBJECTS 255
typedef struct {
  unsigned char equipmentType;
  DDateTimeType sDSMTimeStamp;
  Position3D  refPos;
  PositionalAccuracy refPosXYConf;
  unsigned char numDetectedObjects;
  DetectedObjectCommonData objects[NUM_OF_OBJECTS]; // numDetectedObjects 갯수 만큼만...
} recognition_data_type;



typedef struct {
  v2x_intf_hdr_type hdr;
  recognition_data_type data;
} v2x_recognition_msg_type;

typedef struct {
  v2x_intf_hdr_type hdr;
  
  // TODO: Add situation data
  // situation_data_type data;
} v2x_situation_msg_type;


typedef struct {
  v2x_intf_hdr_type hdr;
  
  // TODO: Add negotiation data
  // negotiation_data_type data;
} v2x_negotiation_msg_type;

typedef union {
  v2x_intf_hdr_type hdr;
  v2x_recognition_msg_type  recog;
  v2x_situation_msg_type    situ;
  v2x_negotiation_msg_type  nego;
} v2x_intf_msg_type;

