import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from serial import Serial
from serial.serialutil import SerialException
import tf2_ros

class SerialOdometryNode(Node):
    def __init__(self):
        super().__init__('vehicle_interface')
        qos = QoSProfile(depth=10)
        # 创建tf2的broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', qos)
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos)
        self.serial_port = Serial('/dev/ttyUSB0', 1000000, timeout=5)

        self.timer_period = 1 / 150  # 150Hz
        self.timer = self.create_timer(self.timer_period, self.read_from_serial)

    def cmd_vel_callback(self, msg):
        try:
            # Prepare the serial message to send the velocities
            serial_msg = 'x {:.2f}\ny {:.2f}\na {:.2f}\n'.format(
                msg.linear.x, msg.linear.y, msg.angular.z)
            self.serial_port.write(serial_msg.encode('utf-8'))
        except SerialException as e:
            self.get_logger().error('Error sending cmd_vel to serial port: %r' % e)

    def read_from_serial(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    # Parse the received odom data
                    parts = line.split()
                    if len(parts) == 12 and parts[0] == 'vx' and parts[2] == 'vy' and parts[4] == 'az' and parts[6] == 'x' and parts[8] == 'y' and parts[10] == 'theta':
                        odom_msg = Odometry()
                        odom_msg.header.stamp = self.get_clock().now().to_msg()
                        odom_msg.header.frame_id = 'odom'
                        odom_msg.child_frame_id = 'base_link'
                        odom_msg.twist.twist.linear.x = float(parts[1])
                        odom_msg.twist.twist.linear.y = float(parts[3])
                        odom_msg.twist.twist.angular.z = float(parts[5])
                        odom_msg.pose.pose.position.x = float(parts[7])
                        odom_msg.pose.pose.position.y = float(parts[9])
                        # Convert theta to quaternion and set it
                        theta = float(parts[11])
                        quat = self.euler_to_quaternion(0, 0, theta)
                        odom_msg.pose.pose.orientation.x = quat[0]
                        odom_msg.pose.pose.orientation.y = quat[1]
                        odom_msg.pose.pose.orientation.z = quat[2]
                        odom_msg.pose.pose.orientation.w = quat[3]
                        self.odom_publisher.publish(odom_msg)

                        tf = TransformStamped()

                        tf.transform.translation.x = float(parts[7])
                        tf.transform.translation.y = float(parts[9])
                        tf.transform.rotation.x = quat[0]
                        tf.transform.rotation.y = quat[1]
                        tf.transform.rotation.z = quat[2]
                        tf.transform.rotation.w = quat[3]

                        # 设置转换的header信息
                        tf.header.stamp = self.get_clock().now().to_msg()
                        tf.header.frame_id = 'odom'
                        tf.child_frame_id = 'base_link'

                        # 发布world到camera的转换
                        self.tf_broadcaster.sendTransform(tf)

        except SerialException as e:
            self.get_logger().error('Error reading from serial port: %r' % e)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Abbreviated conversion from Euler angles to quaternion representation
        import math
    # 将角度转换为弧度
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # 计算四元数
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    serial_odometry_node = SerialOdometryNode()
    rclpy.spin(serial_odometry_node)
    serial_odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
