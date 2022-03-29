from setuptools import setup

package_name = 'omniwheel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'keyboard_controller = omniwheel.controller.keyboard_controller:main',
            'path_visualizer = omniwheel.path_visualizer.path_visualizer:main',
            'path_executor = omniwheel.path_executor.path_executor:main'
        ],
    },
)
