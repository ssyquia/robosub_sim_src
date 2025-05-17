from setuptools import setup

package_name = 'controls_6dof'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='tritonrobosub',
    maintainer_email='robosub@ucsd.edu',
    description='6 Dof Keyboard Control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listenkey = controls_6dof.listenkey:main',
            'publish_vector = controls_6dof.publish_vector:main',
            'publish_string = controls_6dof.publish_string:main',
        ],
    },
)
