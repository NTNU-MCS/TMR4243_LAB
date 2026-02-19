#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

from tmr4243_utilities.utilities import anon

def generate_launch_description():

    vessel_name = "voyager"
    vessel_model = "voyager"

    node_utilities = launch_ros.actions.Node(
        package='tmr4243_utilities',
        executable='utility_node.py',
        name=f"{anon()}utilities",
        namespace=vessel_name,
        output='screen'
    )

    viz_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [
                launch_ros.substitutions.FindPackageShare("cybership_viz"),
                "/launch/viz.launch.py",
            ]
        ),
        launch_arguments=[
            ("vessel_model", launch.substitutions.LaunchConfiguration("vessel_model")),
            ("vessel_name", launch.substitutions.LaunchConfiguration("vessel_name")),
        ],
    )

    group_gui = launch.actions.GroupAction(
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('use_gui', default='true')),
        actions=[viz_launch]
    )

    ld = launch.LaunchDescription()
    ld.add_action(node_utilities)
    ld.add_action(group_gui)

    return ld