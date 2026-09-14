"""Start only online SLAM on top of the already running Bagheera base."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bagheera_share = Path(get_package_share_directory("bagheera_base"))
    slam_share = Path(get_package_share_directory("slam_toolbox"))
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(slam_share / "launch" / "online_async_launch.py")
                ),
                launch_arguments={
                    "autostart": "true",
                    "use_lifecycle_manager": "false",
                    "use_sim_time": "false",
                    "slam_params_file": str(
                        bagheera_share / "config" / "slam.yaml"
                    ),
                }.items(),
            )
        ]
    )
