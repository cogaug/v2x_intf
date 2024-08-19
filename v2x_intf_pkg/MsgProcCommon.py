from v2x_intf_msg.msg import Recognition, Object
import ctypes
import socket
from v2x_intf_pkg.V2XConstants import V2XConstants as v2xconst
import v2x_intf_pkg.FmtCommon as fmtcommon

class MsgProcCommon:
  def __init__(self, logger):
    self.logger = logger

  def parseHeader(self, pkd_data):
    hdr = fmtcommon.v2x_intf_hdr_type()
    ctypes.memmove(ctypes.addressof(hdr), pkd_data[:ctypes.sizeof(fmtcommon.v2x_intf_hdr_type)], ctypes.sizeof(fmtcommon.v2x_intf_hdr_type))
    return hdr.hdr_flag, hdr.msgID, socket.ntohl(hdr.msgLen)


