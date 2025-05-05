from setuptools import find_packages, setup
import os
import glob

package_name = 'high_level_robosub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.*')),
		(os.path.join('share', package_name, 'launch'), glob.glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ed',
    maintainer_email='ed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'force_controls = high_level_robosub.force_controls:main',
			'twist_controls = high_level_robosub.twist_controls:main',
            'twist_to_pose = high_level_robosub.twist_to_pose_updater:main',
        ],
    },
)
