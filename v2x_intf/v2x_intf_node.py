import rclpy as rclpy
from rclpy.node import Node

from case_msgs.msg import Recognition, Objects
import socket


class RecognitionSubscriber(Node):
    def __init__(self):
        super().__init__('recognition_subscriber')
        self.subscription = self.create_subscription(
            Recognition,
            'caselab/recognition',
            self.recognition_callback,
            10)
        self.subscription  # prevent unused variable warning

    def recognition_callback(self, msg):
        print('I heard: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    recognition_subscriber = RecognitionSubscriber()
    rclpy.spin(recognition_subscriber)

    recognition_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()