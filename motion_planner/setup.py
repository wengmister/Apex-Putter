from setuptools import find_packages, setup

package_name = 'motion_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   # 'launch/test_robot_state.launch.py',
                                   'launch/pickplace.launch.py',
                                   'launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyle Puckett',
    maintainer_email='kylepuckett2029@u.northwestern.edu',
    description='Robot State Class/Module with Testing Node; to be integrated \
        into Homework3 for Group3 to keep track of robot state',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_node = motion_planner.pick_node:main',
            'demo_node = motion_planner.demo_node:main',
            'testRS = motion_planner.testRS:main',
            'testMP = motion_planner.testMP:main'
        ],
    },
)
