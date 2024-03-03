import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf_transformations import inverse_matrix, quaternion_matrix, quaternion_from_matrix
from scipy.spatial.transform import Rotation
import numpy as np

class WorldToOdomPublisher(Node):
    def __init__(self):
        super().__init__('world_to_odom_publisher')

        # 创建tf2的buffer和listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 创建tf2的broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.publish_world_to_odom_transform,
            10)

        self.world_to_odom_transform = TransformStamped()
        self.world_to_odom_transform.transform.rotation.w = -1.0

        self.timer_period = 1 / 100  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.pub_tf)

    def pub_tf(self):
        # 设置转换的header信息
        self.world_to_odom_transform.header.stamp = self.get_clock().now().to_msg()
        self.world_to_odom_transform.header.frame_id = 'world'
        self.world_to_odom_transform.child_frame_id = 'odom'

        # 发布world到odom的转换
        self.tf_broadcaster.sendTransform(self.world_to_odom_transform)


    def publish_world_to_odom_transform(self, msg):

        world_to_odom_transform_position_x = []
        world_to_odom_transform_position_y = []
        world_to_odom_transform_position_z = []
        world_to_odom_transform_quaternions = []
        world_to_odom_transform_decision_margins = []

        for detection in msg.detections:
            try:
                # 从tf2 buffer中查找world到apriltag的转换
                world_to_apriltag = self.tf_buffer.lookup_transform('world', 'tag' + str(detection.id) + '_fixed', rclpy.time.Time())

                # 从tf2 buffer中查找odom到apriltag的转换
                odom_to_apriltag = self.tf_buffer.lookup_transform('odom', detection.family + ':' + str(detection.id), rclpy.time.Time())

                # 将TransformStamped消息转换为矩阵
                odom_to_apriltag_matrix = quaternion_matrix([
                    odom_to_apriltag.transform.rotation.x,
                    odom_to_apriltag.transform.rotation.y,
                    odom_to_apriltag.transform.rotation.z,
                    odom_to_apriltag.transform.rotation.w])
                odom_to_apriltag_matrix[0:3, 3] = [
                    odom_to_apriltag.transform.translation.x,
                    odom_to_apriltag.transform.translation.y,
                    odom_to_apriltag.transform.translation.z]

                # 计算矩阵的逆
                apriltag_to_odom_matrix = inverse_matrix(odom_to_apriltag_matrix)

                # 从逆矩阵中提取四元数和平移
                apriltag_to_odom_rot = quaternion_from_matrix(apriltag_to_odom_matrix)
                apriltag_to_odom_trans = apriltag_to_odom_matrix[0:3, 3]

                # 创建TransformStamped消息
                apriltag_to_odom = Pose()
                apriltag_to_odom.position.x = apriltag_to_odom_trans[0]
                apriltag_to_odom.position.y = apriltag_to_odom_trans[1]
                apriltag_to_odom.position.z = apriltag_to_odom_trans[2]
                apriltag_to_odom.orientation.x = apriltag_to_odom_rot[0]
                apriltag_to_odom.orientation.y = apriltag_to_odom_rot[1]
                apriltag_to_odom.orientation.z = apriltag_to_odom_rot[2]
                apriltag_to_odom.orientation.w = apriltag_to_odom_rot[3]

                # 计算world到odom的转换
                world_to_odom = tf2_geometry_msgs.do_transform_pose(
                    apriltag_to_odom,
                    world_to_apriltag
                )

                world_to_odom_transform_single = TransformStamped()

                world_to_odom_transform_single.transform.translation.x = world_to_odom.position.x
                world_to_odom_transform_single.transform.translation.y = world_to_odom.position.y
                world_to_odom_transform_single.transform.translation.z = world_to_odom.position.z
                world_to_odom_transform_single.transform.rotation.x = world_to_odom.orientation.x
                world_to_odom_transform_single.transform.rotation.y = world_to_odom.orientation.y
                world_to_odom_transform_single.transform.rotation.z = world_to_odom.orientation.z
                world_to_odom_transform_single.transform.rotation.w = world_to_odom.orientation.w

                # 设置转换的header信息
                world_to_odom_transform_single.header.stamp = self.get_clock().now().to_msg()
                world_to_odom_transform_single.header.frame_id = 'world'
                world_to_odom_transform_single.child_frame_id = 'odom' + str(detection.id)

                # 发布world到odom的转换
#                self.tf_broadcaster.sendTransform(world_to_odom_transform_single)

                world_to_odom_transform_position_x.append(world_to_odom.position.x)
                world_to_odom_transform_position_y.append(world_to_odom.position.y)
                world_to_odom_transform_position_z.append(world_to_odom.position.z)
                world_to_odom_transform_quaternions.append([world_to_odom.orientation.x, world_to_odom.orientation.y, world_to_odom.orientation.z, world_to_odom.orientation.w])
                world_to_odom_transform_decision_margins.append(detection.decision_margin)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().debug('Could not transform world to odom: %s' % str(e))

        if len(msg.detections) == len(world_to_odom_transform_quaternions) and len(msg.detections) != 0:
            world_to_odom_transform_tmp = TransformStamped()

            weights = [m / sum(world_to_odom_transform_decision_margins) for m in world_to_odom_transform_decision_margins]
            rotations = Rotation.from_quat(world_to_odom_transform_quaternions)
            mean_rotation = rotations.mean(weights=weights)
            mean_quaternion = mean_rotation.as_quat()

            for i in range(len(world_to_odom_transform_position_x)):
                world_to_odom_transform_tmp.transform.translation.x += world_to_odom_transform_position_x[i] * weights[i]
                world_to_odom_transform_tmp.transform.translation.y += world_to_odom_transform_position_y[i] * weights[i]
                world_to_odom_transform_tmp.transform.translation.z += world_to_odom_transform_position_z[i] * weights[i]

            world_to_odom_transform_tmp.transform.rotation.x = mean_quaternion[0]
            world_to_odom_transform_tmp.transform.rotation.y = mean_quaternion[1]
            world_to_odom_transform_tmp.transform.rotation.z = mean_quaternion[2]
            world_to_odom_transform_tmp.transform.rotation.w = mean_quaternion[3]

            self.world_to_odom_transform = world_to_odom_transform_tmp

def main():
    rclpy.init()
    node = WorldToOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
