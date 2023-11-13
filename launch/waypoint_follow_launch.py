from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # cmd_vel_topic = LaunchConfiguration("cmd_vel_topic", default="/cmd_vel")

    package_name = "zed_odom_subscriber"
    joy_params = os.path.join(
        get_package_share_directory(package_name), "config", "joystick.yaml"
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "twist_mux.yaml",
    )

    zed_subcriber = Node(
        package="zed_odom_subscriber",
        executable="zed_odom_subscriber_node",
        # remappings=[("/cmd_vel", cmd_vel_topic)],
        # parameters=[joy_params, {'use_sim_time': use_sim_time}],
    )

    joy_node = Node(  # TO BROADCAST JOY AXES DATA
        package="joy",
        executable="joy_node",
        parameters=[joy_params],
    )

    teleop_node = Node(  # TO INTERPRET JOY AXES DATA TO TWIST CMD_VEL MESSAGE
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params],
        # remappings=[("/cmd_vel", cmd_vel_topic)],
    )

    twist_mux = Node(  # TWIST MULTIPLEXER
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            joy_node,
            twist_mux,
            teleop_node,
            zed_subcriber,
        ]
    )
