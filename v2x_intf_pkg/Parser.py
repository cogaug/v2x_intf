from v2x_intf_msg.msg import Recognition, Object
import struct
import ctypes
from v2x_intf_pkg.V2XConstants import V2XConstants as v2xconst
import v2x_intf_pkg.FmtCommon as fmtcommon
from v2x_intf_pkg.MsgProcRecognition import MsgProcRecognition

class Parser :
  """
  A class to parse incoming data into ROS2 messages according to the msg_type field.

  Attributes:
    logger : Logger
      The logger object to log messages.
  """  
  def __init__(self, logger):
    """
    Constructs the necessary attributes for the Parser object.

    Args:
      logger : Logger
        The logger object to log messages.
    """
    self.logger = logger

  def parse(self, pkd_data):
    """
    Parses the packed data into a Recognition message.

    Args:
      pkd_data : bytes
        The packed data to be parsed.

    Returns:
      Recognition: The parsed Recognition message or None if parsing fails.
    """
    # Ensure pkd_data is bytes
    if not isinstance(pkd_data, bytes):
      self.logger.error("Input data is not bytes")
      return None

    hdr = fmtcommon.v2x_intf_hdr_type()
    ctypes.memmove(ctypes.addressof(hdr), pkd_data[:ctypes.sizeof(fmtcommon.v2x_intf_hdr_type)], ctypes.sizeof(fmtcommon.v2x_intf_hdr_type))

    hdr_flag = hdr.hdr_flag
    msg_type = hdr.msgID
    msg_len = hdr.msgLen
    
    self.logger.info(f'Parsing header: {hdr_flag}, {msg_type}, {msg_len}')
    if hdr_flag is v2xconst.HDR_FLAG:
      self.info('Invalid header flag: %d' % hdr_flag)
      return None
    else :
      header_size = ctypes.sizeof(fmtcommon.v2x_intf_hdr_type)
      data_len = len(pkd_data)
      self.logger.info(f'Parsing header: {hdr_flag}, {msg_type}, {msg_len}, {header_size}, {data_len}')
      if data_len - header_size != msg_len:  # Check message length
        self.logger.error(f"Data size {data_len - header_size} does not match header msg_len {msg_len}")
        return None
      if msg_type == v2xconst.MSG_RECOGNITION :
        return MsgProcRecognition(self.logger).fromV2XMsg(pkd_data)
      
      # TODO : Add support for other message types

      else :
        self.logger.info(f'Not supported message type: {msg_type}')
        return None
    