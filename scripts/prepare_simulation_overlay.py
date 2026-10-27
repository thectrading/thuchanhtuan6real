#!/usr/bin/env python3
"""Keep the TurtleBot model but omit its unused simulated RGB-D sensor.

Assignment 6 reads the real laptop webcam. Rendering robot RGB, depth and a
point cloud competes with that webcam inference for resources without being
used by the behavior controller. LiDAR, cliff sensors, collisions and driver
safety are unchanged. The Nav2/YOLO exercise and /opt/ros are not modified.
"""
import os
from pathlib import Path
import re
import shutil
from ament_index_python.packages import get_package_share_directory


def main():
    prefix = Path.home() / '.cache/turtlebot4_gesture_hri/simulation'
    target = prefix / 'share/turtlebot4_description'
    marker = prefix / 'GENERATED_GESTURE_SIMULATION'
    source = Path(get_package_share_directory('turtlebot4_description')).resolve()
    if source == target.resolve():
        candidates = [Path(p)/'share/turtlebot4_description' for p in
                      os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
                      if p and Path(p).resolve() != prefix.resolve()]
        source = next((p.resolve() for p in candidates
                       if (p/'urdf/sensors/oakd.urdf.xacro').is_file()), None)
        if source is None:
            raise RuntimeError('Cannot locate original TurtleBot 4 description')
    if prefix.exists() and not marker.exists():
        raise RuntimeError(f'Refusing to replace an unowned directory: {prefix}')
    text = (source/'urdf/sensors/oakd.urdf.xacro').read_text()
    text, count = re.subn(r'<sensor\s+name="rgbd_camera"\s+type="rgbd_camera">.*?</sensor>',
        '<!-- Laptop webcam is used by gesture HRI; unused simulated RGB-D omitted. -->',
        text, flags=re.DOTALL)
    if count != 1:
        raise RuntimeError('Expected exactly one RGB-D sensor; inspect simulator description')
    prefix.mkdir(parents=True, exist_ok=True)
    marker.write_text(f'Source: {source}\nOnly modification: omit unused simulated RGB-D sensor.\n')
    shutil.copytree(source, target, dirs_exist_ok=True)
    (target/'urdf/sensors/oakd.urdf.xacro').write_text(text)
    index = prefix/'share/ament_index/resource_index/packages/turtlebot4_description'
    index.parent.mkdir(parents=True, exist_ok=True);index.write_text('')
    print(f'Gesture simulation overlay: {prefix}. Source lab_env.bash and restart simulation.')


if __name__ == '__main__':main()
