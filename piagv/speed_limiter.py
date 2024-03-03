import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpeedLimiter(Node):
    def __init__(self):
        super().__init__('speed_limiter')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_out', 10)

        # 定义最大线速度和角速度
        self.max_linear_speed = 0.4  # 米/秒
        self.max_angular_speed = 0.8  # 弧度/秒

        # 定义最大线加速度和角加速度
        self.max_linear_accel = 0.1  # 米/秒^2
        self.max_angular_accel = 0.2  # 弧度/秒^2

        # 上一次接收到的速度
        self.last_vel = Twist()

        # 上一次接收速度的时间
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # 限制线速度
        linear_accel_x = (msg.linear.x - self.last_vel.linear.x) / dt
        if abs(linear_accel_x) > self.max_linear_accel:
            linear_accel_x = self.max_linear_accel * (linear_accel_x / abs(linear_accel_x))

        new_linear_speed_x = self.last_vel.linear.x + linear_accel_x * dt
        new_linear_speed_x = max(min(new_linear_speed_x, self.max_linear_speed), -self.max_linear_speed)

        linear_accel_y = (msg.linear.y - self.last_vel.linear.y) / dt
        if abs(linear_accel_y) > self.max_linear_accel:
            linear_accel_y = self.max_linear_accel * (linear_accel_y / abs(linear_accel_y))

        new_linear_speed_y = self.last_vel.linear.y + linear_accel_y * dt
        new_linear_speed_y = max(min(new_linear_speed_y, self.max_linear_speed), -self.max_linear_speed)

        # 限制角速度
        angular_accel = (msg.angular.z - self.last_vel.angular.z) / dt
        if abs(angular_accel) > self.max_angular_accel:
            angular_accel = self.max_angular_accel * (angular_accel / abs(angular_accel))

        new_angular_speed = self.last_vel.angular.z + angular_accel * dt
        new_angular_speed = max(min(new_angular_speed, self.max_angular_speed), -self.max_angular_speed)

        # 创建新的Twist消息并发布
        new_vel = Twist()
        new_vel.linear.x = new_linear_speed_x
        new_vel.linear.y = new_linear_speed_y
        new_vel.angular.z = new_angular_speed
        self.publisher.publish(new_vel)

        # 更新上一次的速度和时间
        self.last_vel = new_vel
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    speed_limiter = SpeedLimiter()
    rclpy.spin(speed_limiter)
    speed_limiter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
