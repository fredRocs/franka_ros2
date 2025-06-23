from setuptools import find_packages, setup

package_name = 'trajectory_replayer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/record_trajectory.launch.py']),
        ('share/' + package_name, ['launch/play_trajectory.launch.py']),
        ('share/' + package_name, ['config/fr3_ros_controllers.yaml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='frederic.giraud@balgrist.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'play_trajectory = trajectory_replayer.replay:main',
        ],
    },
)
