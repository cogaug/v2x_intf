import rclpy as rp 
from rclpy.node import Node
from v2x_intf_msg.msg import Recognition, Object
from datetime import datetime, timedelta

class V2XMsgsPub(Node):
    def __init__(self):
        super().__init__('v2x_msg_pub')
        self.timer_period = 0.1
        self.cnt_run = 1
        self.pub_rec = self.create_publisher(Recognition, 'v2x/recognition', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # self.recog = Recognition()

    def timer_callback(self):
        vehicle_position = [(0.321+float(self.cnt_run))%90.0, (0.33232+float(self.cnt_run))%180.0]
        objects = []

        for i in range(36):
            detection_time = self.date_time(30*((self.cnt_run)%10))
            object_position = [(0.434+float(self.cnt_run))%3276.0, (0.343+float(self.cnt_run))%3276.0]
            object_velocity = (0.662+float(self.cnt_run))%163.0
            object_heading =(0.1+float(self.cnt_run))%359.9
            object_class = (self.cnt_run)%9
            recognition_accuracy = self.cnt_run%100
            od = Object(
                detection_time = detection_time,
                object_position = object_position,
                object_velocity = object_velocity,
                object_heading = object_heading,
                object_class = object_class,
                recognition_accuracy = recognition_accuracy    
            )
            objects.append(od)
        self.cnt_run += 1

        re = Recognition(
            vehicle_id = 1,  # int16
            vehicle_time = self.date_time(0), # int32[7], year, month, day, hour, minute, second, microsecond
            vehicle_position = vehicle_position,
            object_data = objects
        )
        self.pub_rec.publish(re)
        self.get_logger().info(f'{self.cnt_run} Publishing: {re}')

    def date_time(self, delay):
        now = datetime.now() - timedelta(milliseconds=delay)
        now_array=[
            now.year,
            now.month,
            now.day,
            now.hour,
            now.minute,
            now.second,
            now.microsecond]
        return now_array
    
def main(args=None):
    rp.init(args=args)
    v2x_msg_pub = V2XMsgsPub()
    rp.spin(v2x_msg_pub)

    v2x_msg_pub.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()