# Launch the syncai_driver_manager node — the UDP bridge to the gait controller.
#
# robot_id is read from the system config INI at launch time (same convention
# as sys_manager.launch.py) and is used as the node namespace, so all the
# node's relative topics and services get the /<robot_id> prefix. This node
# publishes no TF frame names, so it needs no frame-parameter rewriting.

import configparser
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Absolute path so the INI resolves no matter what cwd the launch is started
# from — the old relative path only worked because every entrypoint happened
# to run from the workspace root. ~/robot_ws is the workspace inside the robot
# container, where docker-compose bind-mounts the per-robot instance INI over
# config/system.ini.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"
# Every INI before [system] robot_model existed is a G23's (the workspace's
# launches read the key the same way)
DEFAULT_ROBOT_MODEL = "g23"

logger = launch_logging.get_logger("driver_manager.launch")


def read_robot_id(config_path: str) -> str:
    config = configparser.ConfigParser()
    if not config.read(config_path):
        logger.warning(
            f"System config '{config_path}' not found; "
            f"falling back to robot_id '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    robot_id = config.get("system", "robot_id", fallback="").strip()
    if not robot_id:
        logger.warning(
            f"No [system] robot_id in '{config_path}'; "
            f"falling back to '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    return robot_id


def read_robot_model(config_path: str) -> str:
    """[system] robot_model of the INI, `g23` when the key (or the file) is missing."""
    config = configparser.ConfigParser()
    config.read(config_path)
    model = config.get("system", "robot_model", fallback="").strip().lower()
    if not model:
        logger.warning(
            f"No [system] robot_model in '{config_path}'; reading it as '{DEFAULT_ROBOT_MODEL}'"
        )
        return DEFAULT_ROBOT_MODEL
    return model


def model_params_file(model: str) -> str:
    """params/models/<model>.yaml in this package's share; raises when the model has none.

    The model's file is loaded after params_file and wins key by key. params_file holds only
    what every model shares (the sensor monitor); each model's own keys are in its file:
    g23.yaml has the G23's protocol, UDP endpoints and velocity scales, m20.yaml the M20's
    protocol, endpoint and lidar topics.

    Nothing falls back, as in the workspace's own launches: without its file a robot would come
    up on the code defaults (telemetry 0.0.0.0:50012, every scale 1.0), bind and look healthy,
    and never hear its controller.
    """
    path = os.path.join(
        get_package_share_directory("syncai_driver_manager"), "params", "models", f"{model}.yaml"
    )
    if not os.path.isfile(path):
        raise RuntimeError(
            f"robot_model '{model}' has no params/models/{model}.yaml in syncai_driver_manager; "
            "add one (see params/models/g23.yaml) or fix [system] robot_model"
        )
    return path


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the node.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    params_file = LaunchConfiguration("params_file")
    parameters = [params_file]

    robot_model = read_robot_model(config_path)
    parameters.append(model_params_file(robot_model))

    # No `name=`: the node is constructed as "driver_manager", and the params
    # file uses a bare /** key (rclrs does not expand /**/driver_manager-style
    # wildcards), so it matches in any namespace.
    driver_manager_node = Node(
        package="syncai_driver_manager",
        executable="driver_manager_node",
        namespace=robot_id,
        output="screen",
        parameters=parameters,
    )

    return [driver_manager_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_driver_manager")
    default_params_file = os.path.join(pkg_share, "params", "driver_manager_params.yaml")

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id and robot_model",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the driver_manager parameters YAML file",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
