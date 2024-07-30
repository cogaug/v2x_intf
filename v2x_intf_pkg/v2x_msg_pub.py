import rclpy as rp 
from rclpy.node import Node
from v2x_msgs.msg import Recognition, Objects
from datetime import datetime, timedelta

class V2XMsgsPub(Node):
    def __init__(self):
        super().__init__('v2x_msg_pub')
        self.timer_period = 1.0
        self.cnt_run = 1
        self.pub_rec = self.create_publisher(Recognition, 'v2x_msgs/recognition', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # self.recog = Recognition()

    def timer_callback(self):
        od = Objects(
            detection_time = self.date_time(10*((self.cnt_run)%10)), # int32[7], year, month, day, hour, minute, second, microsecond
            object_position = [(0.434+float(self.cnt_run))%3276.0, (0.343+float(self.cnt_run))%3276.0],
            object_velocity = (0.662+float(self.cnt_run))%163.0,
            object_heading =(0.1+float(self.cnt_run))%359.9,
            object_class = (self.cnt_run)%9,
            recognition_accuracy = (float(self.cnt_run)+50.0)%100.0
        )
        re = Recognition(
            vehicle_id = 1,  # int16
            vehicle_time = self.date_time(0), # int32[7], year, month, day, hour, minute, second, microsecond
            vehicle_position = [(0.321+float(self.cnt_run))%90.0, (0.33232+float(self.cnt_run))%180.0],
            object_data = [od]
        )
        self.pub_rec.publish(re)
        self.get_logger().info('Publishing: "%s"' % re)
        self.cnt_run += 1

    def date_time(self, delay):
        now = datetime.now() + timedelta(milliseconds=delay)
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