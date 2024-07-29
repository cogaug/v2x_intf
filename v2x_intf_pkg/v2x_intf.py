import rclpy as rclpy
from rclpy.node import Node
# from v2x_msgs.msg import Recognition, Objects

from v2x_intf_pkg.tcpconn_man import TcpConnectionManager
from v2x_intf_pkg.recog_sub import RecognitionSubscriber

def main(args=None):
    rclpy.init(args=args)
    connection_manager = TcpConnectionManager()
    recognition_subscriber = RecognitionSubscriber(connection_manager)

    main_node = rclpy.create_node('main_node')

    try:
        # Main loop for your node
        while rclpy.ok():
            # Receive data from the server asynchronously
            if connection_manager.obu_connected:
                received_data = connection_manager.receive_data()
                if received_data is not None:
                    main_node.get_logger().info(f'Received from server: {received_data}')

            rclpy.spin_once(main_node, timeout_sec=0.1)  # Spin the main node for a single iteration
            rclpy.spin_once(recognition_subscriber, timeout_sec=0.1)  # Spin the subscriber node for a single iteration
    
    except KeyboardInterrupt:
        main_node.get_logger().info('Shutting down due to keyboard interrupt.')

    finally:
        # Close the connection when done
        connection_manager.close_connection()
        main_node.destroy_node()
        recognition_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()