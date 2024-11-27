import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tutlebot3_slamNav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
		(os.path.join('share', package_name,'confg'), glob('confg/*config.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mainCode = tutlebot3_slamNav.mainCode:main',
			'explorer_node = tutlebot3_slamNav.explorer_node:main',
			'vizualizer = tutlebot3_slamNav.vizualizer:main'
        ],
    },
)
