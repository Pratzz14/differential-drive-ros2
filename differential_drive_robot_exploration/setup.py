from glob import glob
from setuptools import find_packages, setup


package_name = 'differential_drive_robot_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Robot model maintainer',
    maintainer_email='maintainer@example.com',
    description='Autonomous frontier exploration and mapping benchmark.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'exploration_mission = differential_drive_robot_exploration.exploration_runner:main',
            'run_exploration_benchmark = differential_drive_robot_exploration.benchmark:main',
        ],
    },
)
