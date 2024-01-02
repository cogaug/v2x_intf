import rclpy as rclpy
from rclpy.node import Node

from case_msgs.msg import Recognition, Objects
import socket
import threading
import select

obu_ip = '192.168.2.100' # TODO : MATCH IT!
obu_port = 9201 # TODO : MATCH IT!


class TcpConnectionManager:
    def __init__(self):
        self.obu_ip = '192.168.2.100' # TODO : MATCH IT!
        self.obu_port = 9201
        self.obu_connected = False
        self.receive_buffer = b''
        self.lock = threading.Lock()
        self.client_socket = None

        try :
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.obu_ip, self.obu_port))
            self.obu_connected = True
        except Exception as e:
            print('Error:', str(e))
            self.obu_connected = False

    def send_data(self, data):
        if self.obu_connected :
            with self.lock:
                try:
                    # Serialize and send the data to the server
                    serialized_data = self.serialize_data(data)  # Implement serialization function
                    self.client_socket.send(serialized_data)
                    
                except Exception as e:
                    print('Error:', str(e))
                    return None


    def receive_data(self):
        with self.lock:
            ready_to_read, _, _ = select.select([self.client_socket], [], [], 0.1)
            if ready_to_read:
                received_data = self.client_socket.recv(1024)
                self.response_buffer += received_data
                return self.response_buffer.decode()
            return None

    def close_connection(self):
        with self.lock:
            self.client_socket.close()

    def serialize_data(self, data):
        # Implement serialization logic for your data here
        # For example, you can use pickle or another serialization method
        # Return the serialized data as bytes
        return bytes(str(data), 'utf-8')


class RecognitionSubscriber(Node):
    def __init__(self, connection_manager):
        super().__init__('recognition_subscriber')
        self.subscription = self.create_subscription(
            Recognition,
            'caselab/recognition',
            self.recognition_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.connection_manager = connection_manager

    def convertToV2XMsg(self, msg):
        # TODO : convert to V2X msg
        pass

    def convertFromV2XMsg(self, msg):
        # TODO : convert from V2X msg
        pass

    def recognition_callback(self, msg):
        print('Received recognition message:', msg)
        data = self.convertToV2XMsg(msg)
        # Send the received message data to the server over the shared TCP connection
        response = self.connection_manager.send_data(data)
        if response:
            print('Received from server:', response)


def main(args=None):
    rclpy.init(args=args)
    connection_manager = TcpConnectionManager()
    recognition_subscriber = RecognitionSubscriber(connection_manager)
    # TODO Add other nodes

    # received_data = connection_manager.receive_data()
    # if received_data is not None:
    #     print('Received from server:', received_data)

    # rclpy.spin(recognition_subscriber)


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