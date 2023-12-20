# import rclpy as rclpy
# from rclpy.node import Node

# class RecognitionSubscriber(Node):
#     def __init__(self):
#         super().__init__('recognition_subscriber')
#         self.subscription = self.create_subscription(
#             # Recognition,
#             'caselab/recognition',
#             self.recognition_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def recognition_callback(self, msg):
#         # self.get_logger().info('I heard: "%s"' % msg.data)
#         # Here we have to manipulate the recognition data
#         print('I heard: "%s"' % msg.data)

# def main(args=None):
#     rclpy.init(args=args)
#     recognition_subscriber = RecognitionSubscriber()
#     rclpy.spin(recognition_subscriber)

#     recognition_subscriber.destroy_node()
#     rclpy.shutdown()

def main():
    print('Hi from v2x_subscriber.')


if __name__ == '__main__':
    main()