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

    topic_transform_node = Node(
        package='robot_simulation',
        executable='topic_transform',
        name='topic_transform',
        output='screen',
        parameters=[{'use_sim_time': True}]
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
    ld.add_action(topic_transform_node)
    ld.add_action(autonomy_stack_go2)
    return ld