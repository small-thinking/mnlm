from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch folder
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'flask', 'rospy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='2237303+yxjiang@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_receiver_node = robot_arm.command_receiver:main',
            'command_dispatcher_node = robot_arm.command_dispatcher:main'
        ],
    },
)
