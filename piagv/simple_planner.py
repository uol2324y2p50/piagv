import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
import tf_transformations
import math

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')

        # 创建Twist消息发布者
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # 创建tf2缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_callback,
            10)

        # 创建定时器，每秒调用一次timer_callback
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.x_set = -0.5
        self.y_set = 0.0
        self.a_set = 0.0


    def timer_callback(self):
        try:
            # 查找world到base_link的变换
            trans = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())

            pitch, roll, yaw = quaternion_to_euler(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)

            # 从变换中获取x轴的平移分量
            x_translation = (trans.transform.translation.x - self.x_set) * -0.5
            y_translation = (trans.transform.translation.y - self.y_set) * -0.5
            a_translation = (yaw - self.a_set) * -0.5


            # 创建Twist消息
            twist = Twist()
            twist.linear.x = math.cos(-yaw) * x_translation - math.sin(-yaw) * y_translation
            twist.linear.y = math.sin(-yaw) * x_translation + math.cos(-yaw) * y_translation
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = a_translation

            # 发布Twist消息到/cmd_vel
            self.publisher_.publish(twist)

        except Exception as e:
            self.get_logger().debug('Could not transform world to base_link: %s' % str(e))

    def pose_callback(self, msg):
        self.x_set = msg.pose.position.x
        self.y_set = msg.pose.position.y
        pitch, roll, yaw = quaternion_to_euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.a_set = yaw


def quaternion_to_euler(x, y, z, w):
    # 计算各个轴上的旋转
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z  # 返回欧拉角

def main(args=None):
    rclpy.init(args=args)
    simple_planner = SimplePlanner()
    rclpy.spin(simple_planner)
    simple_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
