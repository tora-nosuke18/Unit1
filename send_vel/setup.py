import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'send_vel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tora',
    maintainer_email='tiger.tora1210@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover = send_vel.rover:main',
            'odom_calc = send_vel.odom_calc:main',
            'hand_twist = send_vel.hand_twist:main'

        ],
    },
)
