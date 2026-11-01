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
from scipy.spatial.transform import Rotation as R

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

        while rclpy.ok():
            try:
                self.tf_bl_to_velo=self.tf_buffer.lookup_transform('base_link', 'velodyne', rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(f'Waiting for base_link -> velodyne: {ex}')
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            try:
                self.tf_velo_to_bf=self.tf_buffer.lookup_transform('velodyne', 'base_footprint', rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(f'Waiting for velodyne -> base_footprint: {ex}')
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            self.pose_bl_to_velo = Pose()
            self.pose_bl_to_velo.position.x = self.tf_bl_to_velo.transform.translation.x
            self.pose_bl_to_velo.position.y = self.tf_bl_to_velo.transform.translation.y
            self.pose_bl_to_velo.position.z = self.tf_bl_to_velo.transform.translation.z
            self.pose_bl_to_velo.orientation = self.tf_bl_to_velo.transform.rotation

            # self.pose_velo_to_bf = Pose()
            # self.pose_velo_to_bf.position.x = self.tf_velo_to_bf.transform.translation.x
            # self.pose_velo_to_bf.position.y = self.tf_velo_to_bf.transform.translation.y
            # self.pose_velo_to_bf.position.z = self.tf_velo_to_bf.transform.translation.z
            # self.pose_velo_to_bf.orientation = self.tf_velo_to_bf.transform.rotation
            self.p_velo_to_bf = np.array([
                self.tf_velo_to_bf.transform.translation.x,
                self.tf_velo_to_bf.transform.translation.y,
                self.tf_velo_to_bf.transform.translation.z,
            ])
            q_velo_to_bf = np.array([
                self.tf_velo_to_bf.transform.rotation.x,
                self.tf_velo_to_bf.transform.rotation.y,
                self.tf_velo_to_bf.transform.rotation.z,
                self.tf_velo_to_bf.transform.rotation.w,
            ])
            self.R_velo_to_bf = R.from_quat(q_velo_to_bf)


            self.get_logger().info('Both static TFs ready')
            break
        
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


    def sync_callback(self, cloud_msg, input_odom_msg):

        tf_map_to_bl = TransformStamped()
        tf_map_to_bl.header = input_odom_msg.header
        tf_map_to_bl.header.frame_id = 'map'
        tf_map_to_bl.child_frame_id = 'base_link'
        tf_map_to_bl.transform.translation.x = input_odom_msg.pose.pose.position.x
        tf_map_to_bl.transform.translation.y = input_odom_msg.pose.pose.position.y
        tf_map_to_bl.transform.translation.z = input_odom_msg.pose.pose.position.z
        tf_map_to_bl.transform.rotation = input_odom_msg.pose.pose.orientation

        pose_map_to_velo = do_transform_pose(self.pose_bl_to_velo, tf_map_to_bl)
        p = pose_map_to_velo.position
        q = pose_map_to_velo.orientation
        curr_xyz = np.array([p.x, p.y, p.z])
        curr_quat = np.array([q.w, q.x, q.y, q.z])

        if not self.init:
            self.xyz0 = curr_xyz.copy()
            self.quat0 = curr_quat.copy()
            self.init = True

        pos_rel = curr_xyz - self.xyz0
        q_rel = qmult(qinverse(self.quat0), curr_quat)

        output_odom_msg = Odometry()
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

        # connect tf tree
        # p_map_to_velo = pos_rel
        # R_map_to_velo = R.from_quat(q_rel)
        # p_map_to_bf = p_map_to_velo + R_map_to_velo.apply(self.p_velo_to_bf)
        # q_map_to_bf = (R_map_to_velo * self.R_velo_to_bf).as_quat()
        # tf_map_to_bf = TransformStamped()
        # tf_map_to_bf.header.stamp = input_odom_msg.header.stamp
        # tf_map_to_bf.header.frame_id = 'map'
        # tf_map_to_bf.child_frame_id = 'base_footprint'
        # tf_map_to_bf.transform.translation.x = p_map_to_bf[0]
        # tf_map_to_bf.transform.translation.y = p_map_to_bf[1]
        # tf_map_to_bf.transform.translation.z = p_map_to_bf[2]
        # tf_map_to_bf.transform.rotation.w = q_map_to_bf[0]
        # tf_map_to_bf.transform.rotation.x = q_map_to_bf[1]
        # tf_map_to_bf.transform.rotation.y = q_map_to_bf[2]    
        # tf_map_to_bf.transform.rotation.z = q_map_to_bf[3]

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

        q_rel_inv = qinverse(q_rel)
        pos_rel_inv = -quat2mat(q_rel).T @ pos_rel
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

        # self.tf_broadcaster.sendTransform([tf_map_to_bf, tf_map_to_sensor])
        self.tf_broadcaster.sendTransform([tf_map_to_sensor])
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

