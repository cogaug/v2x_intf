import rclpy as rclpy
import asyncio
import threading
from rclpy.executors import MultiThreadedExecutor
from v2x_intf_pkg.tcpconn_man import TcpConnectionManager
from v2x_intf_pkg.recog_sub import RecognitionSubscriber
from v2x_intf_pkg.recog_pub import RecognitionPublisher


def main(args=None):
  rclpy.init(args=args)
  connection_manager = TcpConnectionManager()
  recognition_publisher = RecognitionPublisher(connection_manager)
  recognition_subscriber = RecognitionSubscriber(connection_manager)

  executor = MultiThreadedExecutor()
  executor.add_node(recognition_publisher)
  executor.add_node(recognition_subscriber)

  loop = asyncio.get_event_loop()

  def ros_spin():
    try :
      executor.spin()
    finally:
      executor.shutdown()

    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.start()

    loop = asyncio.get_event_loop()

  try:
    loop.run_forever()
  except KeyboardInterrupt:
    recognition_publisher.get_logger().info('Shutting down due to keyboard interrupt.')

  finally:
    # Close the connection when done
    connection_manager.close_connection()
    recognition_publisher.destroy_node()
    recognition_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()