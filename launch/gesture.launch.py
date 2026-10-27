"""Start human gesture control AFTER the arena is prepared and webcam is ready."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    recognizer = Node(package='turtlebot4_gesture_hri', executable='gesture_recognition_node',
        prefix=[LC('python_executable')], output='screen', parameters=[LC('params'),
            {'camera_index': ParameterValue(LC('camera_index'), value_type=int),
             'model': LC('model'), 'show_window': ParameterValue(LC('show_window'), value_type=bool)}])
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share('turtlebot4_gesture_hri'), 'config', 'gesture.yaml')),
        DeclareLaunchArgument('python_executable', default_value='/usr/bin/python3'),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('show_window', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('model', default_value=os.path.expanduser('~/.cache/turtlebot4_gesture_hri/gesture_recognizer.task')),
        RegisterEventHandler(OnProcessExit(target_action=recognizer, on_exit=[
            EmitEvent(event=Shutdown(reason='Webcam recognizer exited: stopping gesture control'))])),
        Node(package='turtlebot4_gesture_hri', executable='robot_behavior_node',
             parameters=[LC('params')], output='screen'), recognizer])
