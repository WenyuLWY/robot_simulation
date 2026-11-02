#!/usr/bin/env python3

import rospy
import tf2_ros

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudRegistrationNode:
    def __init__(self):
        # 接收并缓存 TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.registered_pc_pub = rospy.Publisher(
            '/registered_scan',
            PointCloud2,
            queue_size=10
        )

        self.odom_pub = rospy.Publisher(
            '/state_estimation',
            Odometry,
            queue_size=10
        )

        self.pc_sub = rospy.Subscriber(
            '/velodyne_points',
            PointCloud2,
            self.pc_callback,
            queue_size=10
        )

        rospy.loginfo('PointCloud Registration Node started.')

    def pc_callback(self, msg):
        # 查询将点云坐标转换到 map 坐标系的最新可用 TF
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rospy.Time(0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as e:
            rospy.logwarn('TF query failed: %s', e)
            return

        # 将点云转换到 map 坐标系
        try:
            pc_in_map = do_transform_cloud(msg, transform)

            # 保留点云的采集时间
            pc_in_map.header.stamp = msg.header.stamp
            pc_in_map.header.frame_id = 'map'

        except Exception as e:
            rospy.logerr('PointCloud transform failed: %s', e)
            return

        self.registered_pc_pub.publish(pc_in_map)

        # 发布点云传感器在 map 坐标系下的位姿
        odom = Odometry()
        odom.header.stamp = transform.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = msg.header.frame_id

        translation = transform.transform.translation
        odom.pose.pose.position.x = translation.x
        odom.pose.pose.position.y = translation.y
        odom.pose.pose.position.z = translation.z

        odom.pose.pose.orientation = transform.transform.rotation

        # 未估计速度，twist 保持默认零值
        self.odom_pub.publish(odom)


def main():
    rospy.init_node('pointcloud_registration_node')
    node = PointCloudRegistrationNode()
    rospy.spin()


if __name__ == '__main__':
    main()