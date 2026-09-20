"""One-command bringup (backward compatible).

Includes the three phases in order with generous fixed timers:

    1. simulation.launch.py    — Gazebo + bridge + robot spawn
    2. localization.launch.py  — depth pipeline + map server + AMCL
    3. navigation.launch.py    — Nav2 controller/planner/behavior/navigator

Prefer scripts/run_system.sh instead: it starts the same phases as separate
processes and waits for real readiness gates (odom flowing, map -> odom
available, /scan streaming from live MiDaS depth) between phases. Fixed
timers cannot know when the Android phone starts streaming, so this file
uses conservative delays: without a phone streaming, the navigation phase
will still fail to activate the global costmap.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(launch_file: str) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file))
    )


def generate_launch_description():
    sim = Path(get_package_share_directory("indoor_nav_gazebo")) / "launch" / "simulation.launch.py"
    loc = Path(get_package_share_directory("indoor_nav_costmap")) / "launch" / "localization.launch.py"
    nav = Path(get_package_share_directory("indoor_nav_costmap")) / "launch" / "navigation.launch.py"

    return LaunchDescription(
        [
            _include(sim),
            TimerAction(period=5.0, actions=[_include(loc)]),
            TimerAction(period=45.0, actions=[_include(nav)]),
        ]
    )
