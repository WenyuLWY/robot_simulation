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

    ld= LaunchDescription()
    ld.add_action(gazebo_velodyne)
    return ld