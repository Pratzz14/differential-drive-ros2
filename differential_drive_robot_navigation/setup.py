from glob import glob
from setuptools import find_packages, setup


package_name = 'differential_drive_robot_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Pratik Mahankal',
    maintainer_email='pratik.mahankal14@gmail.com',
    description='Seeded autonomous-navigation benchmark for the differential-drive robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_runner = differential_drive_robot_navigation.mission_runner:main',
        ],
    },
)
