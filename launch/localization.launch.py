import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource,AnyLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def static_tf(x, y, z, roll, pitch, yaw, parent, child, name):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            str(x), str(y), str(z),
            str(roll), str(pitch), str(yaw),
            parent,
            child
        ],
        output='screen'
    )

def generate_launch_description():

    rtabmap=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_simulation'), 'launch', 'lidar3d.launch.py')
            ),
            launch_arguments={
                'lidar_topic':   'velodyne_points',
                'frame_id':      'sensor',
                'use_sim_time':  'true',
                'localization':  'false',
                'rtabmap_viz':   'false',
                # 'imu_topic': '/imu/data',
                # 'odom_topic': '/odom',
            }.items(),
        )



    static_transform_node = LaunchDescription([
            static_tf(0, 0, -0.118, 0, 0, 0, 'vehicle', 'base_link', 'tf_vehicle_to_footprint'),
            static_tf(-0.2, 0, 0, 0, 0, 0, 'sensor', 'vehicle', 'tf_camera'),
        ])

    trajectory_node = Node(
        package='mogi_trajectory_server',
        executable='mogi_trajectory_server',
        name='mogi_trajectory_server',
        parameters=[{'reference_frame_id': 'map',
                    'robot_frame_id': 'sensor',
                    'use_sim_time': True}]
    )


    ld= LaunchDescription()
    ld.add_action(rtabmap)
    ld.add_action(static_transform_node)
    ld.add_action(trajectory_node)
    return ld