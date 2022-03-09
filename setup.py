import os
from glob import glob
from setuptools import setup

package_name = 'trajcontrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mariana Bernardes',
    maintainer_email='bernardes@unb.br',
    description='Needle trajectory compensation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_sensor = trajcontrol.virtual_sensor:main',
            'virtual_robot = trajcontrol.virtual_robot:main',
            'virtual_UI = trajcontrol.virtual_UI:main',
            'sensor_processing = trajcontrol.sensor_processing:main',
            'smart_template = trajcontrol.smart_template:main',
            'estimator_node = trajcontrol.estimator_node:main',
            'controller_node = trajcontrol.controller_node:main',
            'save_file = trajcontrol.save_file:main',
            'insertion_node = trajcontrol.insertion_node:main',
        ],
    },
)
