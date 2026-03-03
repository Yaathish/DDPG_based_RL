from setuptools import find_packages, setup

package_name = 'lidar_obstacle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='batch9',
    maintainer_email='batch9@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detection = lidar_obstacle.obstacle_detection:main',
            'obstacle_avoidance = lidar_obstacle.obstacle_avoidance:main',
            'goal_navigator = lidar_obstacle.goal_navigator:main',
        ],
    },
)
