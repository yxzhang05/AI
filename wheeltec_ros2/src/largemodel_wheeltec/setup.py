from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'largemodel_wheeltec'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WHEELTEC',
    maintainer_email='wheeltec@todo.todo',
    description='Voice and text control for WHEELTEC robot using large language models',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_service = largemodel.action_service_wheeltec:main',
        ],
    },
)
