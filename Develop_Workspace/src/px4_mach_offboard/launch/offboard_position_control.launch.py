#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import json

def generate_launch_description():
    package_dir = get_package_share_directory('px4_mach_offboard')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    waypoint_list = [
        [37.5478915, 127.1194249, 100.0, float('nan')],
        [37.5474712, 127.1186771, 100.0, float('nan')],
        [37.5468944, 127.1186654, 100.0, float('nan')],
    ] 

    waypoints_str = json.dumps(waypoint_list)  

    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        # Node(
        #     package='px4_mach_offboard',
        #     namespace='px4_mach_offboard',
        #     executable='visualizer',
        #     name='visualizer'
        # ),
        Node(
            package='px4_mach_offboard',
            namespace='px4_mach_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        Node(
            package='px4_mach_offboard',
            namespace='px4_mach_offboard',
            executable='control',
            name='control',
            prefix='gnome-terminal --',
        ),
        Node(
            package='px4_mach_offboard',
            namespace='px4_mach_offboard',
            executable='offboard_control',
            name='offboard_control',
            parameters= [{'takeoff_altitude': 3.1}]
            # parameters= [{'waypoints': waypoints_str}, {'takeoff_altitude': 3.0}]
        ),
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        # )
    ])
