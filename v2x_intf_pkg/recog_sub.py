import rclpy
from rclpy.node import Node
from v2x_intf_msg.msg import Recognition
from v2x_intf_pkg.msg_conv import RecognitionMsg
from datetime import datetime

class RecognitionSubscriber(Node):
  def __init__(self, connection_manager):
    super().__init__('recognition_subscriber')

    self.equipmentType = 2 # unknown (0), rsu (1), obu (2)
    self.subscription = self.create_subscription(
            Recognition,
            'v2x/recognition',
            self.recognition_callback,
            10
        )
        
    # self.subscription  # prevent unused variable warning
    self.connection_manager = connection_manager
    self.get_logger().info('Recognition subscriber initialized')
    
  def recognition_callback(self, msg):
    self.get_logger().info(f'(ROS2->) Received recognition message at {datetime.now()} : {msg}')
    recogMsg = RecognitionMsg(self.get_logger())
    try:
      data = recogMsg.toV2XMsg(msg)
      # Send the received message data to the server over the shared TCP connection
      if data :
        self.get_logger().info(f'(->V2X) Send recognition message at {datetime.now()}')
        self.connection_manager.send_data(data)
    except Exception as e:
      self.get_logger().error(f'Error processing recognition message: {e}')
