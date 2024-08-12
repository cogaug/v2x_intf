import rclpy as rclpy
import asyncio
import threading
import argparse
from rclpy.executors import MultiThreadedExecutor
from v2x_intf_pkg.v2x_conn_man import IntfConnManager
from v2x_intf_pkg.v2x_intf_recog_sub import RecognitionSubscriber
from v2x_intf_pkg.v2x_intf_recog_pub import RecognitionPublisher


def main(args=None):
  rclpy.init(args=args)

  parser = argparse.ArgumentParser(description='V2X OBU Interface')
  parser.add_argument('--obu-ip', type=str, default='127.0.0.1', help='The IP address of the OBU (default: 127.0.0.1)')
  parser.add_argument('--obu-port', type=int, default=9201, help='The port of the OBU (default: 9201)')
  parsed_args = parser.parse_args()
  
  connection_manager = IntfConnManager(obu_ip=parsed_args.obu_ip, obu_port=parsed_args.obu_port)
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
    loop.stop()
    ros_thread.join()

if __name__ == '__main__':
    main()