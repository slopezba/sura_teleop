import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def environment_for_robot(robot_namespace):
    description_package = f"{robot_namespace}_description"
    config_path = os.path.join(
        get_package_share_directory(description_package),
        "config",
        "bringup_description.yaml",
    )

    with open(config_path, "r", encoding="utf-8") as config_file:
        bringup_description = yaml.safe_load(config_file) or {}

    robot_data = bringup_description.get("robot", {})
    profile_robot_name = str(robot_data.get("name", "")).strip("/")
    if profile_robot_name and profile_robot_name != robot_namespace:
        raise RuntimeError(
            f"robot.name '{profile_robot_name}' from '{config_path}' must match "
            f"launch argument robot_namespace '{robot_namespace}'."
        )

    environment = str(robot_data.get("environment", "")).strip()
    if environment not in ("sim", "real"):
        raise RuntimeError(
            f"robot.environment in '{config_path}' must be 'sim' or 'real', "
            f"got '{environment}'."
        )

    return environment


def config_for_robot(package_share, robot_namespace, environment):
    config_name = f"teleop_params_{robot_namespace}_{environment}.yaml"
    config_path = os.path.join(package_share, "config", config_name)
    if not os.path.isfile(config_path):
        raise RuntimeError(
            "Teleop configuration file not found. Expected "
            f"'{config_path}' for robot_namespace='{robot_namespace}' "
            f"and environment='{environment}'."
        )
    return config_path


def rewrite_namespace_values(value, robot_namespace):
    if isinstance(value, dict):
        return {
            key: rewrite_namespace_values(item, robot_namespace)
            for key, item in value.items()
        }
    if isinstance(value, list):
        return [rewrite_namespace_values(item, robot_namespace) for item in value]
    if isinstance(value, str):
        return value.replace("/sura/", f"/{robot_namespace}/")
    return value


def namespaced_config(config_file, robot_namespace):
    with open(config_file, "r", encoding="utf-8") as input_file:
        config = yaml.safe_load(input_file) or {}

    node_config = config.get("sura_teleop")
    if node_config is None:
        node_config = config.get(f"/{robot_namespace}/sura_teleop")
    if node_config is None:
        raise RuntimeError(
            f"Teleop configuration '{config_file}' must define parameters for "
            "'sura_teleop'."
        )

    node_config = rewrite_namespace_values(node_config, robot_namespace)
    output_config = {f"/{robot_namespace}/sura_teleop": node_config}
    output_file = f"/tmp/sura_teleop_{robot_namespace}_{Path(config_file).name}"
    with open(output_file, "w", encoding="utf-8") as output:
        yaml.safe_dump(output_config, output, default_flow_style=False)
    return output_file


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    if not robot_namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")

    environment = LaunchConfiguration("environment").perform(context).strip()
    if not environment:
        environment = environment_for_robot(robot_namespace)
    if environment not in ("sim", "real"):
        raise RuntimeError(
            f"Launch argument 'environment' must be 'sim' or 'real', got '{environment}'."
        )

    package_share = get_package_share_directory("sura_teleop")
    params_file = namespaced_config(
        config_for_robot(package_share, robot_namespace, environment),
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
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("environment", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
