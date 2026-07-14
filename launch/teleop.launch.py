from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


def namespaced_config(config_file, robot_namespace):
    text = Path(config_file).read_text(encoding="utf-8")
    if robot_namespace:
        text = text.replace("sura_teleop:", f"/{robot_namespace}/sura_teleop:", 1)
        text = text.replace("/sura/", f"/{robot_namespace}/")

    output_file = f"/tmp/sura_teleop_{robot_namespace or 'root'}_{Path(config_file).name}"
    Path(output_file).write_text(text, encoding="utf-8")
    return output_file


def config_for_namespace(package_share, robot_namespace, teleop_profile):
    namespace = robot_namespace.lower()
    if "bluerov" in namespace:
        config_name = "teleop_params_bluerov.yaml"
    else:
        profile = teleop_profile.lower() or "sim"
        if profile not in ("sim", "real"):
            raise RuntimeError(
                f"Launch argument 'teleop' must be 'sim' or 'real', got '{teleop_profile}'."
            )
        config_name = f"teleop_params_cirtesub_{profile}.yaml"
    return os.path.join(package_share, "config", config_name)


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    teleop_profile = LaunchConfiguration("teleop").perform(context).strip()
    package_share = get_package_share_directory("sura_teleop")
    params_file = namespaced_config(
        config_for_namespace(package_share, robot_namespace, teleop_profile),
        robot_namespace,
    )

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
        namespace=robot_namespace,
        output="screen",
        parameters=[params_file],
    )

    return [
        joy_node,
        teleop_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="sura"),
        DeclareLaunchArgument("teleop", default_value="sim"),
        OpaqueFunction(function=launch_setup),
    ])
