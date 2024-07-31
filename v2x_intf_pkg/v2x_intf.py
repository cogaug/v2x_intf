import rclpy as rclpy
from v2x_intf_pkg.tcpconn_man import TcpConnectionManager
from v2x_intf_pkg.recog_sub import RecognitionSubscriber
from v2x_intf_pkg.recog_pub import RecognitionPublisher

def main(args=None):
  rclpy.init(args=args)
  connection_manager = TcpConnectionManager()
  recognition_subscriber = RecognitionSubscriber(connection_manager)
  recognition_publisher = RecognitionPublisher(connection_manager)

  main_node = rclpy.create_node('main_node')

  try:
    # Main loop for your node
    while rclpy.ok():
      rclpy.spin_once(main_node, timeout_sec=0.1)  # Spin the main node for a single iteration
      rclpy.spin_once(recognition_subscriber, timeout_sec=0.1)  # Spin the subscriber node for a single iteration
      rclpy.spin_once(recognition_publisher, timeout_sec=0.1)

  except KeyboardInterrupt:
    main_node.get_logger().info('Shutting down due to keyboard interrupt.')

  finally:
    # Close the connection when done
    connection_manager.close_connection()
    main_node.destroy_node()
    recognition_subscriber.destroy_node()
    recognition_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()