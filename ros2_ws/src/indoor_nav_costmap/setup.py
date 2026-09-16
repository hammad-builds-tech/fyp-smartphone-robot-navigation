from setuptools import find_packages, setup

package_name = 'indoor_nav_costmap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hammad',
    maintainer_email='hammad@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_obstacle_node = indoor_nav_costmap.depth_obstacle_node:main',
            'depth_to_occupancy_grid = indoor_nav_costmap.depth_to_occupancy_grid:main',
            'depth_to_pointcloud = indoor_nav_costmap.depth_to_pointcloud:main',
            'depth_to_scan = indoor_nav_costmap.depth_to_scan:main',
            'pointcloud_to_grid = indoor_nav_costmap.pointcloud_to_grid:main',
        ],
    },
)
