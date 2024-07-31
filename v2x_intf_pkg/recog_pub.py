# This file to add the publish function for the v2x interface messages.
import rclpy
from rclpy.node import Node
from v2x_msgs.msg import Recognition
from v2x_intf_pkg.msg_conv import Parser

class RecognitionPublisher(Node):
  def __init__(self, connection_manager):
    super().__init__('recognition_publisher')
    self.connection_manager = connection_manager
    self.parser = Parser(self.get_logger())
    self.recognition_publisher = self.create_publisher(Recognition, 'v2x_msgs/r_recognition', 10)   
    self.timer = self.create_timer(0.1, self.timer_callback)

  def timer_callback(self):
    if self.connection_manager.obu_connected:
      received_data = self.connection_manager.receive_data()
      if received_data is not None:
        self.get_logger().info(f'Received from server: {received_data}\n\n')
                
        # Parse the received data
        recognition_data = self.parser.parse(received_data)

        # Convert parsed data to Recognition message
        recognition_msg = Recognition()
        # Assuming recognition_data contains the necessary fields for the Recognition message
        # recognition_msg.field1 = recognition_data.field1  # Modify these fields based on actual message fields
        # recognition_msg.field2 = recognition_data.field2
        # Add more fields as necessary

        # Publish the Recognition message
        # self.recognition_publisher.publish(recognition_msg)
        # self.get_logger().info(f'Published recognition message: {recognition_msg}') 