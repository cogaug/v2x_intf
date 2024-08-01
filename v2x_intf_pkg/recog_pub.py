# This file to add the publish function for the v2x interface messages.
import rclpy
from rclpy.node import Node
from v2x_intf_msg.msg import Recognition
from v2x_intf_pkg.msg_conv import Parser
import asyncio
from datetime import datetime

class RecognitionPublisher(Node):
  def __init__(self, connection_manager):
    super().__init__('recognition_publisher')
    self.connection_manager = connection_manager
    self.parser = Parser(self.get_logger())
    self.recognition_publisher = self.create_publisher(Recognition, 'v2x/r_recognition', 10)   
    self.loop = asyncio.get_event_loop()
    self.loop.create_task(self.receive_data_async())
    self.get_logger().info('Recognition publisher initialized')

  async def receive_data_async(self):
    while rclpy.ok():
      if self.connection_manager.obu_connected:
        # Run the blocking receive_data method in a separate thread
        received_data = await self.loop.run_in_executor(None, self.connection_manager.receive_data)
        if received_data is not None:
          self.get_logger().info(f'(V2X->) received data at {datetime.now()}')          
          # Parse the received data
          recognition_msg = self.parser.parse(received_data)
          if recognition_msg is not None:
            # Publish the Recognition message
            self.get_logger().info(f'(->ROS2) Publish recognition message at {datetime.now()}: {recognition_msg}')    
            self.recognition_publisher.publish(recognition_msg)

  def shutdown(self):
    self.connection_manager.close_connection()
    self.destroy_node()