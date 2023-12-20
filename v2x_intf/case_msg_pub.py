import rclpy as rp 
from rclpy.node import Node
from case_msgs.msg import Recognition, Objects
import datetime as d

class CaseMsgsPub(Node):
    def __init__(self):
        super().__init__('case_msg_pub')
        self.timer_period = 1.0
        self.cnt_run = 1
        self.pub_rec = self.create_publisher(Recognition, 'caselab/recognition', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # self.recog = Recognition()

    def timer_callback(self):
        od = Objects(
            object_position = [0.434+float(self.cnt_run), 0.343+float(self.cnt_run)],
            object_velocity = float(0.662+self.cnt_run),
            object_class = self.cnt_run+40,
            recognition_accuracy = float(self.cnt_run+50.0)
        )
        re = Recognition(
            vehicle_id = 1,
            datetime = self.date_time(),
            vehicle_pose = [0.321+float(self.cnt_run), 0.33232+float(self.cnt_run)],
            vehicle_velocity = float(self.cnt_run+0.412),
            object_data = [od]
        )
        # re.object_data.append(od)
        self.pub_rec.publish(re)
        self.get_logger().info('Publishing: "%s"' % re)

    def date_time(self):
        now = d.datetime.now()
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
    case_msgs_pub = CaseMsgsPub()
    rp.spin(case_msgs_pub)

    case_msgs_pub.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()