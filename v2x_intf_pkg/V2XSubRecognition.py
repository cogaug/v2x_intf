import rclpy
from rclpy.node import Node
from v2x_intf_msg.msg import Recognition
from v2x_intf_pkg.MsgProcRecognition import MsgProcRecognition
from datetime import datetime

class SubRecognition(Node):
  def __init__(self, connection_manager):
    super().__init__('V2X_Recognition')

    self.equipmentType = 2 # unknown (0), rsu (1), obu (2)
    self.subscription = self.create_subscription(
            Recognition,
            'v2x/recognition',
            self._callback,
            10
        )
        
    self.subscription  # prevent unused variable warning
    self.connection_manager = connection_manager
    self.get_logger().info('Recognition subscriber initialized')
    
  def _callback(self, msg):
    self.get_logger().info(f'(ROS2->) Received recognition message at {datetime.now()} : {msg}')
    msg_proc = MsgProcRecognition(self.get_logger())
    try:
      data = msg_proc.toV2XMsg(msg)
      # Send the received message data to the server over the shared TCP connection
      if data :
        if self.connection_manager.obu_connected :
          self.get_logger().info(f'(->V2X) Send recognition message at {datetime.now()}')
          self.connection_manager.send_data(data)
        else:
          self.get_logger().error('Error: Connection not open, ignore TOPIC')
          self.connection_manager.open_connection()
    except Exception as e:
      self.get_logger().error(f'Error processing recognition message: {e}')
