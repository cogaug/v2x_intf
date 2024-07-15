import rclpy as rclpy
from rclpy.node import Node

from v2x_msgs.msg import Recognition, Objects

from tcpconn_man import TcpConnectionManager
from recog_sub import RecognitionSubscriber

def main(args=None):
    rclpy.init(args=args)
    connection_manager = TcpConnectionManager()
    recognition_subscriber = RecognitionSubscriber(connection_manager)
    # TODO Add other nodes

    try:
        node = rclpy.create_node('main_node')  # Create a main node instance
        # Main loop for your node
        while rclpy.ok():
            # Receive data from the server asynchronously
            if connection_manager.obu_connected:
                received_data = connection_manager.receive_data()
                if received_data is not None:
                    print('Received from server:', received_data)

            # TODO: Process other tasks or messages here
            rclpy.spin_once(recognition_subscriber)  # Spin ROS 2 for a single iteration
    
    except KeyboardInterrupt:
        pass

    # Close the connection when done
    connection_manager.close_connection()

    # TODO Add other nodes to destroy_node
    recognition_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()