from v2x_intf_msg.msg import Recognition, Object
import struct
import ctypes
from v2x_intf_pkg.V2XConstants import V2XConstants as v2xconst
import v2x_intf_pkg.FmtCommon as fmtcommon
from v2x_intf_pkg.MsgProcRecognition import MsgProcRecognition
from v2x_intf_pkg.MsgProcCommon import MsgProcCommon

class Parser :
  def __init__(self, logger):

    self.logger = logger
    self.msgProcCommon = MsgProcCommon(self.logger)
    self.msgProcRecognition = MsgProcRecognition(self.logger)

  def parse(self, pkd_data):
    # Ensure pkd_data is bytes
    if not isinstance(pkd_data, bytes):
      self.logger.error("Input data is not bytes")
      return None

    hdr_flag, msg_type, msg_len = self.msgProcCommon.parseHeader(pkd_data)    
    if hdr_flag is v2xconst.HDR_FLAG:
      self.info('Invalid header flag: %d' % hdr_flag)
      return None
    else :
      header_size = ctypes.sizeof(fmtcommon.v2x_intf_hdr_type)
      data_len = len(pkd_data)
      self.logger.info(f'Received data length: {data_len}, header size: {header_size}, message length field: {msg_len}')
      if data_len - header_size != msg_len:  # Check message length\
        self.logger.info(f'Invalid message length: {data_len - header_size} != {msg_len}')
        return None
      if msg_type == v2xconst.MSG_RECOGNITION :
        return self.msgProcRecognition.fromV2XMsg(pkd_data)
      
      # TODO : Add support for other message types
      # elif msg_type == v2xconst.MSG_SITUATION :
      #   return self.msgProcSituation.fromV2XMsg(pkd_data)

      else :
        self.logger.info(f'Not supported message type: {msg_type}')
        return None
    