from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # Launch MAVROS 2
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14550@',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'plugin_whitelist': ['*'],
                'plugin_denylist': ['guided_target'],  # disable guided_target plugin
            }],
        ),

        # Static TF: map → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # Static TF: odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),

        # Static TF: base_link → base_link_frd
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_link_frd'],
            output='screen'
        ),
    ])