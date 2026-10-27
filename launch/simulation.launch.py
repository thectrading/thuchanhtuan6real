"""Official TurtleBot 4 robot/controllers in an open 8 m arena; no Nav2 or SLAM."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    pkg = share('turtlebot4_gesture_hri'); tb = share('turtlebot4_ignition_bringup')
    resources = os.pathsep.join([os.path.dirname(share('turtlebot4_description')),
        os.path.dirname(share('irobot_create_description')), os.path.join(tb, 'worlds'),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    world = os.path.join(pkg, 'worlds', 'gesture_arena.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='gesture_arena', choices=['gesture_arena']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', os.pathsep.join([
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''), os.environ.get('LD_LIBRARY_PATH', '')])),
        ExecuteProcess(cmd=['ign', 'gazebo', world, '-r', '-v', '2', '--gui-config',
                            os.path.join(pkg, 'config', 'gui.config')], condition=IfCondition(LC('gui'))),
        ExecuteProcess(cmd=['ign', 'gazebo', world, '-r', '-s', '-v', '2'], condition=UnlessCondition(LC('gui'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(tb, 'launch', 'turtlebot4_spawn.launch.py')),
            launch_arguments={'model': 'lite', 'rviz': 'false', 'nav2': 'false', 'slam': 'false',
                              'localization': 'false', 'use_sim_time': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gesture_clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])])
