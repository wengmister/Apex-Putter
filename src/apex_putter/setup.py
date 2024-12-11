from setuptools import find_packages, setup

package_name = 'apex_putter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', ['launch/vision.launch.xml',
                                                'launch/demo.launch.py',
                                                'launch/pickplace.launch.py',
                                                'launch/calibration.launch.xml',
                                                'launch/demo_cp.launch.py']),
        ('share/' + package_name + '/config/', ['config/apriltag.rviz',
                                                'config/atag_ros.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhengyang Kris Weng',
    maintainer_email='wengmister@gmail.com',
    description='ME495 Final Project Fall 2024 Group 3 - Apex Putter: a mini-golf robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_node = apex_putter.demo_node:main',
            'pick_node = apex_putter.pick_node:main',
            'vision = apex_putter.vision:main',
            'yolo = apex_putter.yolo:main',
            'demo = apex_putter.demo:main',
            'calibration = apex_putter.calibration:main',
            'demo_cp = apex_putter.demo_cp:main'
        ],
    },
)
