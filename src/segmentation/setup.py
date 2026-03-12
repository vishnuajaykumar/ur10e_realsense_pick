from setuptools import setup
import os
from glob import glob

package_name = 'segmentation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vishnu Ajaykumar',
    maintainer_email='vishnuajaykumar@gmail.com',
    description='YOLOv8-seg inference node for ROS2',
    license='AGPL-3.0',
    entry_points={
        'console_scripts': [
            'yolo_segmentation_node = segmentation.yolo_segmentation_node:main',
        ],
    },
)
