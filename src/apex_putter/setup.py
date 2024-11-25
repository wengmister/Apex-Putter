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
        ('share/' + package_name + '/launch/', ['launch/vision.launch.xml']),
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
        ],
    },
)
