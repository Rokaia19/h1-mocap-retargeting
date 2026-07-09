import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'h1_mocap_retarget'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pretty Sevde',
    maintainer_email='sevdebogazkesen@gmail.com',
    description='Retargets Vicon mocap joint-angle trials onto the Unitree H1_2 (handless) model for Rviz playback',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'retarget_node = h1_mocap_retarget.retarget_node:main',
        ],
    },
)
