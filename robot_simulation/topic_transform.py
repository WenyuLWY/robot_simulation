#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudRegistrationNode(Node):
    def __init__(self):
        super().__init__('pointcloud_registration_node')

        # 接收并缓存 TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pc_callback,
            10
        )

        self.registered_pc_pub = self.create_publisher(
            PointCloud2,
            '/registered_scan',
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/state_estimation',
            10
        )

        self.get_logger().info(
            'PointCloud Registration Node started.'
        )

    def pc_callback(self, msg: PointCloud2):
        # 查询将点云坐标转换到 map 坐标系的最新可用 TF
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=msg.header.frame_id,
                time=Time()
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warning(f'TF query failed: {e}')
            return

        # 将点云转换到 map 坐标系
        try:
            pc_in_map = do_transform_cloud(msg, transform)

            # 保留点云的采集时间
            pc_in_map.header.stamp = msg.header.stamp
            pc_in_map.header.frame_id = 'map'

        except Exception as e:
            self.get_logger().error(
                f'PointCloud transform failed: {e}'
            )
            return

        self.registered_pc_pub.publish(pc_in_map)

        # 发布点云传感器在 map 坐标系下的位姿
        odom = Odometry()
        odom.header.stamp = transform.header.stamp
        odom.header.frame_id = transform.header.frame_id
        odom.child_frame_id = msg.header.frame_id

        # translation 是 Vector3，position 是 Point，逐字段赋值
        translation = transform.transform.translation
        odom.pose.pose.position.x = translation.x
        odom.pose.pose.position.y = translation.y
        odom.pose.pose.position.z = translation.z

        # 两者都是 Quaternion，可以直接赋值
        odom.pose.pose.orientation = transform.transform.rotation

        # 未估计速度，twist 保持默认零值
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRegistrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()