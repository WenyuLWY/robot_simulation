import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose


from tf2_ros import TransformBroadcaster
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_geometry_msgs import do_transform_pose

import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
from transforms3d.quaternions import quat2mat, qinverse, qmult
from transforms3d.affines import compose
from tf_transformations import quaternion_from_matrix,translation_from_matrix

class TopicTransformer(Node):

    def __init__(self):
        super().__init__('topic_transformer')

        # qos
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        
        # subscriber
        self.cloud_sub = message_filters.Subscriber(
            self, PointCloud2, '/velodyne_points', qos_profile=qos
        )
        # /odom: odom->base_footprint
        # /odom/ground_truth: map->base_link
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, '/odom/ground_truth', qos_profile=qos
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.cloud_sub, self.odom_sub],
            queue_size=10,
            slop=0.05  # 允许 50ms 误差
        )
        self.ts.registerCallback(self.sync_callback)


        # Transformed cloud publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/registered_scan',
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/state_estimation',
            10
        )

        self.first_odom = None
        self.T0 = None
        self.quat0 = None
        self.init=False


    def odom_to_matrix(self, odom: Odometry):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation

        # transforms3d 使用 wxyz
        quat = [q.w, q.x, q.y, q.z]
        R = quat2mat(quat)
        T = compose([p.x, p.y, p.z], R, [1, 1, 1])
        return T, quat

    def pose_to_matrix(self, pose: Pose):
        t = pose.position
        q = pose.orientation

        quat = [q.w, q.x, q.y, q.z]  # wxyz
        R = quat2mat(quat)
        T = compose([t.x, t.y, t.z], R, [1, 1, 1])

        return T

    def sync_callback(self, cloud_msg, input_odom_msg):

        # print("Received synchronized messages:")
        # print(f"PointCloud2 timestamp: {cloud_msg.header.stamp.sec}.{cloud_msg.header.stamp.nanosec}")
        # print(f"Odometry timestamp: {input_odom_msg.header.stamp.sec}.{input_odom_msg.header.stamp.nanosec}")
        #    
        try:
            tf_bl_to_bf = self.tf_buffer.lookup_transform(
                'base_link',
                'base_footprint',   
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Cannot transform base_footprint -> base_link: {ex}'
            )
            return

        # pose_in_base_link = PoseStamped()
        # pose_in_base_link.header = odom_msg.header
        # pose_in_base_link.header.frame_id = 'map'
        # pose_in_base_link.pose = odom_msg.pose.pose

        pose_bl_to_bf = Pose()
        pose_bl_to_bf.position.x = tf_bl_to_bf.transform.translation.x
        pose_bl_to_bf.position.y = tf_bl_to_bf.transform.translation.y
        pose_bl_to_bf.position.z = tf_bl_to_bf.transform.translation.z
        pose_bl_to_bf.orientation = tf_bl_to_bf.transform.rotation

        tf_map_to_bl = TransformStamped()
        tf_map_to_bl.header = input_odom_msg.header
        tf_map_to_bl.header.frame_id = 'map'
        tf_map_to_bl.child_frame_id = 'base_link'
        tf_map_to_bl.transform.translation.x = input_odom_msg.pose.pose.position.x
        tf_map_to_bl.transform.translation.y = input_odom_msg.pose.pose.position.y
        tf_map_to_bl.transform.translation.z = input_odom_msg.pose.pose.position.z
        tf_map_to_bl.transform.rotation = input_odom_msg.pose.pose.orientation
        
        pose_map_to_bf = do_transform_pose(pose_bl_to_bf, tf_map_to_bl)
        if not self.init:
            self.T0 = self.pose_to_matrix(pose_map_to_bf)
            self.init = True

        T= self.pose_to_matrix(pose_map_to_bf)
        T_rel = np.linalg.inv(self.T0) @ T
        pos_rel = translation_from_matrix(T_rel)
        q_rel = quaternion_from_matrix(T_rel) 



        output_odom_msg = Odometry()
        # output_odom_msg.header.stamp = self.get_clock().now().to_msg() 
        output_odom_msg.header.stamp = input_odom_msg.header.stamp

        output_odom_msg.header.frame_id = 'map'
        output_odom_msg.child_frame_id = "sensor"
        output_odom_msg.pose.pose.position.x = pos_rel[0]
        output_odom_msg.pose.pose.position.y = pos_rel[1]
        output_odom_msg.pose.pose.position.z = pos_rel[2]
        output_odom_msg.pose.pose.orientation.w = q_rel[0]
        output_odom_msg.pose.pose.orientation.x = q_rel[1]
        output_odom_msg.pose.pose.orientation.y = q_rel[2]
        output_odom_msg.pose.pose.orientation.z = q_rel[3]
        output_odom_msg.twist.twist.linear.x = 0.0
        output_odom_msg.twist.twist.angular.z = 0.0
        # output_odom_msg.pose.pose.position.x = pose_in_bf.pose.position.x
        # output_odom_msg.pose.pose.position.y = pose_in_bf.pose.position.y
        # output_odom_msg.pose.pose.position.z = pose_in_bf.pose.position.z
        # output_odom_msg.pose.pose.orientation = pose_in_bf.pose.orientation
        # output_odom_msg.twist.twist.linear.x = 0.0
        # output_odom_msg.twist.twist.angular.z = 0.0

        tf_map_to_bf = TransformStamped()
        tf_map_to_bf.header.stamp = input_odom_msg.header.stamp
        tf_map_to_bf.header.frame_id = 'map'
        tf_map_to_bf.child_frame_id = 'base_footprint'
        tf_map_to_bf.transform.translation.x = pos_rel[0]
        tf_map_to_bf.transform.translation.y = pos_rel[1]
        tf_map_to_bf.transform.translation.z = pos_rel[2]
        tf_map_to_bf.transform.rotation.w = q_rel[0]
        tf_map_to_bf.transform.rotation.x = q_rel[1]
        tf_map_to_bf.transform.rotation.y = q_rel[2]    
        tf_map_to_bf.transform.rotation.z = q_rel[3]

        tf_map_to_sensor = TransformStamped()
        tf_map_to_sensor.header.stamp = input_odom_msg.header.stamp
        tf_map_to_sensor.header.frame_id = 'map'
        tf_map_to_sensor.child_frame_id = 'sensor'
        tf_map_to_sensor.transform.translation.x = pos_rel[0]
        tf_map_to_sensor.transform.translation.y = pos_rel[1]
        tf_map_to_sensor.transform.translation.z = pos_rel[2]
        tf_map_to_sensor.transform.rotation.w = q_rel[0]
        tf_map_to_sensor.transform.rotation.x = q_rel[1]
        tf_map_to_sensor.transform.rotation.y = q_rel[2]
        tf_map_to_sensor.transform.rotation.z = q_rel[3]

        T_rel_inv = np.linalg.inv(T_rel)
        pos_rel_inv = translation_from_matrix(T_rel_inv)
        q_rel_inv = quaternion_from_matrix(T_rel_inv) 
        tf_sensor_to_map = TransformStamped()
        tf_sensor_to_map.header.stamp = input_odom_msg.header.stamp
        tf_sensor_to_map.header.frame_id = 'sensor'
        tf_sensor_to_map.child_frame_id = 'map'
        tf_sensor_to_map.transform.translation.x = pos_rel_inv[0]
        tf_sensor_to_map.transform.translation.y = pos_rel_inv[1]
        tf_sensor_to_map.transform.translation.z = pos_rel_inv[2]
        tf_sensor_to_map.transform.rotation.w = q_rel_inv[0]
        tf_sensor_to_map.transform.rotation.x = q_rel_inv[1]
        tf_sensor_to_map.transform.rotation.y = q_rel_inv[2]
        tf_sensor_to_map.transform.rotation.z = q_rel_inv[3]

        cloud_out = do_transform_cloud(
            cloud_msg,
            tf_sensor_to_map
        )
        
        cloud_out.header.frame_id = 'map'
        cloud_out.header.stamp = cloud_msg.header.stamp

        self.tf_broadcaster.sendTransform([tf_map_to_bf, tf_map_to_sensor])
        self.odom_pub.publish(output_odom_msg)
        self.cloud_pub.publish(cloud_out)

    

    
def main(args=None):

    rclpy.init(args=args)

    node = TopicTransformer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

