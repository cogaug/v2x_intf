import rclpy as rclpy
from rclpy.node import Node

from case_msgs.msg import Recognition, Objects
import socket

obu_ip = '192.168.2.100' # TODO : MATCH IT!
obu_port = 9201 # TODO : MATCH IT!

class RecognitionSubscriber(Node):
    def __init__(self):
        super().__init__('recognition_subscriber')
        self.subscription = self.create_subscription(
            Recognition,
            'caselab/recognition',
            self.recognition_callback,
            10)
        self.subscription  # prevent unused variable warning

    def convertToV2XMsg(self, msg):
        # TODO : convert to V2X msg
        pass

    def convertFromV2XMsg(self, msg):
        # TODO : convert from V2X msg
        pass

    def recognition_callback(self, msg):
        self.convertToV2XMsg(msg)
        print("I heard:", msg)


def main(args=None):
    rclpy.init(args=args)
    recognition_subscriber = RecognitionSubscriber()
    rclpy.spin(recognition_subscriber)

    # client_socket.close()
    recognition_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
