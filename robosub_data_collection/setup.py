from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robosub_data_collection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Comprehensive data collection package for robosub simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collector = robosub_data_collection.comprehensive_data_collector:main',
            'gate_controller = robosub_data_collection.gate_controller:main',
            'simple_collector = robosub_data_collection.simple_camera_collector:main',
        ],
    },
) 