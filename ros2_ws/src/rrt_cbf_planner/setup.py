from setuptools import find_packages, setup

package_name = 'rrt_cbf_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/cbf_rrt.launch.py',
            'launch/cbf_rrt_real.launch.py',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/ethan_rrt.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ethan Nguyen-Huu',
    maintainer_email='enguyenhuu@ucsd.edu',
    description='RRT* global planner + CBF-QP safety controller for TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_node = rrt_cbf_planner.rrt_node:main',
            'cbf_controller_node = rrt_cbf_planner.cbf_controller_node:main',
            'obstacle_sim_node = rrt_cbf_planner.obstacle_sim_node:main',
            'optitrack_obstacle_node = rrt_cbf_planner.optitrack_obstacle_node:main',
        ],
    },
)
