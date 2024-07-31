import rclpy as rclpy
import threading

from v2x_intf_pkg.tcpconn_man import TcpConnectionManager
from v2x_intf_pkg.recog_sub import RecognitionSubscriber
from v2x_intf_pkg.recog_pub import RecognitionPublisher

def spin_node(node):
  rclpy.spin(node)

def main(args=None):
  rclpy.init(args=args)
  connection_manager = TcpConnectionManager()
  recognition_publisher = RecognitionPublisher(connection_manager)
  recognition_subscriber = RecognitionSubscriber(connection_manager)

   # Start spinning each node in a separate thread
  threads = []
  for node in [recognition_publisher, recognition_subscriber]:
    thread = threading.Thread(target=spin_node, args=(node,))
    thread.start()
    threads.append(thread)

  try:
    for thread in threads:
      thread.join()  # Wait for all threads to complete  except KeyboardInterrupt:
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