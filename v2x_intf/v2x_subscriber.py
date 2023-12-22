import rclpy as rclpy
from rclpy.node import Node
from case_msgs.msg import Recognition, Objects
import socket
obu_ip = '192.168.2.100' # TODO : MATCH IT!
obu_port = 1000 # TODO : MATCH IT!

# Create a TCP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

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
        # TODO : Open socket & send data to OBU
        pass
# #             int32 vehicle_id
# # int32[] datetime
# # float32[] vehicle_pose
# # float32 vehicle_velocity
# # case_msgs/Objects[] object_data

#         print("I heard:", msg)
#         # self.get_logger().info('I heard:

def main(args=None):
    rclpy.init(args=args)
    recognition_subscriber = RecognitionSubscriber()
    rclpy.spin(recognition_subscriber)

    recognition_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
