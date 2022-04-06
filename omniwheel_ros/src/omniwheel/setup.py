from setuptools import setup
from setuptools import find_packages

from glob import glob
import os

package_name = 'omniwheel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        (os.path.join('share', package_name), glob('launch/display.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pscontroller = omniwheel.controller.ps_controller:main',
            'teensy_node = omniwheel.teensy_node:main',
            'path_vis = omniwheel.path_visualizer.path_visualizer:main',
            'path_exec = omniwheel.path_executor.path_executor:main',
            'frame_publisher = omniwheel.frame_publisher:main'
        ],
    },
)
