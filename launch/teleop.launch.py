from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("sura_teleop")
    params_file = os.path.join(package_share, "config", "teleop_params.yaml")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id": 0,
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
        }],
    )

    teleop_node = Node(
        package="sura_teleop",
        executable="cirtesub_teleop",
        name="sura_teleop",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])
