"""The operator side: mission manager plus console.

Matthew's simulation is launched separately (terrain.launch.py or
simulation.launch.py) because it owns the rover and this owns the operator.
Two launches, two halves, and either can be restarted without the other.

One conflict to know about: his terrain launch starts gps_start_publisher,
which publishes the same /robot_gps_start_location topic this manager does.
Run one or the other. See integration/INTEGRATION.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    origin_lat = LaunchConfiguration('origin_latitude')
    origin_lon = LaunchConfiguration('origin_longitude')
    return LaunchDescription([
        DeclareLaunchArgument('origin_latitude', default_value='38.42287240335025',
                              description='DEM centre, must match s1_navigation'),
        DeclareLaunchArgument('origin_longitude', default_value='-110.78495572815902'),
        DeclareLaunchArgument('console', default_value='true'),
        DeclareLaunchArgument('publish_local_waypoints', default_value='false',
                              description=('publish targets as a local PoseArray on /waypoints '
                                           'as well as degrees on /gps_waypoints. Needed in the '
                                           'flat world, which runs no gps_waypoint_converter')),
        DeclareLaunchArgument('expected_targets', default_value='3',
                              description='how many pairs gps_waypoint_converter accepts'),
        DeclareLaunchArgument('planner_world_limit_m', default_value='1000.0',
                              description=('how far out astar_planner will plan. The flat '
                                           'obstacle world is 10.0; the terrain world is 1000.0')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(package='s1_gui', executable='mission_manager', output='screen',
             parameters=[{'origin_latitude': origin_lat,
                          'origin_longitude': origin_lon,
                          'publish_local_waypoints':
                              LaunchConfiguration('publish_local_waypoints'),
                          'expected_targets': LaunchConfiguration('expected_targets'),
                          'planner_world_limit_m':
                              LaunchConfiguration('planner_world_limit_m'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='s1_gui', executable='console', output='screen',
             condition=IfCondition(LaunchConfiguration('console')),
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
