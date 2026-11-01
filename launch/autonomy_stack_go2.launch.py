import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource,AnyLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    static_transform_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vehicle_to_footprint_tf',
            output='screen',
            arguments=['--x', '0.099',
                        '--y', '0',
                        '--z', '-0.328',
                        '--yaw', '0',
                        '--pitch', '0',
                        '--roll', '0',
                        '--frame-id', 'vehicle',
                        '--child-frame-id', 'base_footprint',
                    ],
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
    ld.add_action(autonomy_stack_go2)
    # ld.add_action(trajectory_node)
    return ld