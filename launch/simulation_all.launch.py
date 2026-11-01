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

    gazebo_velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_config'),
                'launch',
                'gazebo_velodyne.launch.py'
            )
        ),
        launch_arguments={
        'headless': 'True',
        'world_init_z': '0.275',
    }.items()
    )

    # pointcloud_to_laserscan = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan',
    #     remappings=[
    #         ('cloud_in', '/velodyne_points'),   # 你的 PointCloud2 话题
    #         ('scan', '/scan'),               # slam_toolbox 要的
    #     ],
    #     parameters=[{
    #         'use_sim_time': True,
    #         'target_frame': 'velodyne',
    #         'transform_tolerance': 0.01,
    #         'min_height': -0.1,
    #         'max_height': 0.2,
    #         'angle_min': -3.14159,
    #         'angle_max': 3.14159,
    #         'angle_increment': 0.0087,
    #         'scan_time': 0.1,
    #         'range_min': 0.2,
    #         'range_max': 10.0,
    #         'use_inf': True,
    #     }],
    #     output='screen',
    # )

    # slam= IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('go2_config'),
    #             'launch',
    #             'slam.launch.py'
    #         )
    #     )
    # )

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

    topic_transform_node = Node(
        package='robot_simulation',
        executable='topic_transform',
        name='topic_transform',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    trajectory_node = Node(
        package='mogi_trajectory_server',
        executable='mogi_trajectory_server',
        name='mogi_trajectory_server',
        parameters=[{'reference_frame_id': 'map',
                    'robot_frame_id': 'sensor',
                    'use_sim_time': True}]
    )

    autonomy_stack_go2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_simulation'),
                'launch',
                'system_simulation.launch.xml'
            )
        )
    )

    ld= LaunchDescription()
    ld.add_action(gazebo_velodyne)
    # ld.add_action(pointcloud_to_laserscan)
    ld.add_action(rtabmap)
    ld.add_action(static_transform_node)
    ld.add_action(trajectory_node)
    # ld.add_action(slam)
    ld.add_action(topic_transform_node)
    ld.add_action(autonomy_stack_go2)
    return ld