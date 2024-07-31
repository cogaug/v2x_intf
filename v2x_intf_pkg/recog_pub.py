# This file to add the publish function for the v2x interface messages.
import rclpy
from rclpy.node import Node
from v2x_msgs.msg import Recognition
from v2x_intf_pkg.msg_conv import Parser
import asyncio

class RecognitionPublisher(Node):
  def __init__(self, connection_manager):
    super().__init__('recognition_publisher')
    self.connection_manager = connection_manager
    self.parser = Parser(self.get_logger())
    self.recognition_publisher = self.create_publisher(Recognition, 'v2x_msgs/r_recognition', 10)   
    # self.timer = self.create_timer(0.1, self.timer_callback)
    self.loop = asyncio.get_event_loop()
    self.loop.create_task(self.receive_data_async())

  async def receive_data_async(self):
    while rclpy.ok():
      if self.connection_manager.obu_connected:
        # Run the blocking receive_data method in a separate thread
        received_data = await self.loop.run_in_executor(None, self.connection_manager.receive_data)
        if received_data is not None:
          self.get_logger().info(f'Received from server: {received_data}')
                  
          # Parse the received data
          recognition_data = self.parser.parse(received_data)

          if recognition_data is not None:
            # Convert parsed data to Recognition message
            recognition_msg = Recognition()
            # Assuming recognition_data contains the necessary fields for the Recognition message
            # recognition_msg.field1 = recognition_data.field1  # Modify these fields based on actual message fields
            # recognition_msg.field2 = recognition_data.field2
            # Add more fields as necessary

            # Publish the Recognition message
            self.recognition_publisher.publish(recognition_msg)
            self.get_logger().info(f'Published recognition message: {recognition_msg}')    