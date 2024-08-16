import rclpy as rclpy
import asyncio
import threading
import argparse
from rclpy.executors import MultiThreadedExecutor
from v2x_intf_pkg.IntfConnManager import IntfConnManager
from v2x_intf_pkg.SubRecognition import SubRecognition
from v2x_intf_pkg.V2XReceiver import V2XReceiver


def main(args=None):
  rclpy.init(args=args)

  parser = argparse.ArgumentParser(description='V2X OBU Interface')
  parser.add_argument('--obu-ip', type=str, default='127.0.0.1', help='The IP address of the OBU (default: 127.0.0.1)')
  parser.add_argument('--obu-port', type=int, default=9201, help='The port of the OBU (default: 9201)')
  parsed_args = parser.parse_args()
  
  connection_manager = IntfConnManager(obu_ip=parsed_args.obu_ip, obu_port=parsed_args.obu_port)
  recognition_subscriber = SubRecognition(connection_manager) # ROS2의 /v2x/recognition topic을 구독하는 노드
  v2xreceiver = V2XReceiver(connection_manager) # OBU에서 받은 데이터를 분석하여 ROS2 메시지를 발행하는 노드

  executor = MultiThreadedExecutor()
  executor.add_node(recognition_subscriber)
  executor.add_node(v2xreceiver)

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
    v2xreceiver.get_logger().info('Shutting down due to keyboard interrupt.')

  finally:
    # Close the connection when done
    connection_manager.close_connection()
    recognition_subscriber.destroy_node()
    v2xreceiver.destroy_node()
    rclpy.shutdown()
    loop.stop()
    ros_thread.join()

if __name__ == '__main__':
    main()