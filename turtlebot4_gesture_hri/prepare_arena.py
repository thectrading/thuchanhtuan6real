"""Move the simulated charging dock to a corner BEFORE gesture control starts.

This is scene preparation, not teleoperation: no robot cmd_vel, pose reset or
gesture is published. The robot stays at the arena center; all motion later
requires a human selection plus Thumb_Up.
"""
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import DockStatus
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def enable_simulated_reverse(node):
    """Allow a 0.5 m reverse leg on this Gazebo robot, never a real driver."""
    get = node.create_client(GetParameters, '/motion_control/get_parameters')
    set_params = node.create_client(SetParameters, '/motion_control/set_parameters')
    if not get.wait_for_service(timeout_sec=10) or not set_params.wait_for_service(timeout_sec=10):
        raise RuntimeError('Simulation motion_control parameters unavailable')
    req = GetParameters.Request(); req.names = ['use_sim_time']
    future = get.call_async(req); rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if (not future.done() or future.result() is None or not future.result().values
            or not future.result().values[0].bool_value):
        raise RuntimeError('Refusing to configure reverse travel: driver must use simulation time')
    request = SetParameters.Request(parameters=[Parameter(name='safety_override',
        value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='backup_only'))])
    future = set_params.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done() or future.result() is None or not all(r.successful for r in future.result().results):
        raise RuntimeError('Could not enable simulated long reverse travel')
    req.names = ['safety_override']; future = get.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done() or future.result() is None or future.result().values[0].string_value != 'backup_only':
        raise RuntimeError('Reverse policy readback failed')


def main():
    rclpy.init(); node = Node('prepare_gesture_arena'); state = [None]
    sub = node.create_subscription(DockStatus, '/dock_status', lambda m: state.__setitem__(0, m),
                                   qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 90
        while state[0] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if state[0] is None:
            raise RuntimeError('No /dock_status; start gesture simulation first')
        response = subprocess.run(['ign', 'service', '-s', '/world/gesture_arena/set_pose',
            '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '10000', '--req',
            'name: "standard_dock" position {x: 3.0 y: 3.0 z: 0.0} orientation {w: 1.0}'],
            check=True, capture_output=True, text=True, timeout=15)
        if 'true' not in response.stdout.lower():
            raise RuntimeError('Gazebo did not confirm dock scene relocation: ' + response.stdout)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state[0] is not None and not state[0].is_docked:
                enable_simulated_reverse(node)
                print('Arena ready: dock at (3,3), robot undocked; Gazebo driver backup_only allows long reverse. Other driver safety and LiDAR remain active. No robot velocity command was sent.')
                return
        raise RuntimeError('Dock status did not become undocked; do not start motion')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
