from setuptools import find_packages, setup

package_name = 'apex_putter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/pickplace.launch.py',
                                   'launch/demo.launch.py']),

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
            'pick_node = apex_putter.pick_node:main',
            'demo_node = apex_putter.demo_node:main'
        ],
    },
)
