from setuptools import setup

setup(name='robo_virtual_wall', version='0.1.0', packages=['robo_virtual_wall'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robo_virtual_wall']),
                  ('share/robo_virtual_wall', ['package.xml', 'README.md', 'NAV2.md']),
                  ('share/robo_virtual_wall/config', ['config/tape_validation.rviz', 'config/self_mask_20261113.json',
                    'config/dynamic_gripper_20261113.json', 'config/dynamic_gripper_reference_20261113.png',
                    'config/dynamic_gripper_current_20261113.json', 'config/dynamic_gripper_reference_current_20261113.png']),
                  ('share/robo_virtual_wall/launch', ['launch/vision.launch.py', 'launch/operational_tape.launch.py',
                    'launch/observation_tape.launch.py'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Robo-WORK', maintainer_email='robo@example.com',
      description='Striped tape vision and confirmed Nav2 keepout masks', license='Apache-2.0',
      entry_points={'console_scripts': ['virtual_wall = robo_virtual_wall.node:main',
                                      'tape_keepout = robo_virtual_wall.keepout_node:main',
                                      'validate_tape_calibration = robo_virtual_wall.calibrate:main']})
