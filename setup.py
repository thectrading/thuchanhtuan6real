from glob import glob
from os.path import isfile
from setuptools import setup

name = 'turtlebot4_gesture_hri'
setup(name=name, version='0.1.0', packages=[name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml', 'README.md', 'requirements.txt']),
                  *[('share/' + name + '/' + folder, [p for p in glob(folder + '/*') if isfile(p)])
                    for folder in ('launch', 'config', 'worlds', 'docs', 'scripts')],
                  ('share/' + name + '/docs/images', glob('docs/images/*.png'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Tô Văn Chúc', maintainer_email='student@example.com',
      description='MediaPipe webcam hand gesture selection/confirmation control of TurtleBot 4',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'gesture_recognition_node = turtlebot4_gesture_hri.recognition_node:main',
          'robot_behavior_node = turtlebot4_gesture_hri.behavior_node:main',
          'prepare_arena = turtlebot4_gesture_hri.prepare_arena:main']})
