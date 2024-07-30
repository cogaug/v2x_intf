import rclpy
from rclpy.node import Node
from v2x_msgs.msg import Recognition
import struct
import datetime
import math
from v2x_intf_pkg.v2x_const import V2XConstants as v2xconst
from v2x_intf_pkg.v2x_msg_conv import RecognitionMsg

class RecognitionSubscriber(Node):
    def __init__(self, connection_manager):
        super().__init__('recognition_subscriber')

        self.equipmentType = 2 # unknown (0), rsu (1), obu (2)

        self.subscription = self.create_subscription(
            Recognition,
            'v2x_msgs/recognition',
            self.recognition_callback,
            10
        )
        
        # self.subscription  # prevent unused variable warning
        self.connection_manager = connection_manager

    def recognition_callback(self, msg):
        self.get_logger().info('Received recognition message')
        recogMsg = RecognitionMsg()
        try:
            data = recogMsg.toV2XMsg(msg)
            # Send the received message data to the server over the shared TCP connection
            if data :
              self.connection_manager.send_data(data)
              # if response:
              #     self.get_logger().info(f'Received response from server: {response}')
              #     pass
        except Exception as e:
            self.get_logger().error(f'Error processing recognition message: {e}')
