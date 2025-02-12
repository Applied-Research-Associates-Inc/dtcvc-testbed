import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(name="role_name", default_value="ego_vehicle"),
            launch_ros.actions.Node(
                package="dtcvc_competitor",
                executable="dtcvc_scoring_test_competitor",
                name=[
                    "dtcvc_competitor_",
                    launch.substitutions.LaunchConfiguration("role_name"),
                ],
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"use_sim_time": True},
                    {"role_name": launch.substitutions.LaunchConfiguration("role_name")}
                ],
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
