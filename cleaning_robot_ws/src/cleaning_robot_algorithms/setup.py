from setuptools import setup
import os
from glob import glob

package_name = 'cleaning_robot_algorithms'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Path planning algorithms for cleaning robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'astar     = cleaning_robot_algorithms.astar:main',
            'dijkstra  = cleaning_robot_algorithms.dijkstra:main',
            'dwa       = cleaning_robot_algorithms.dwa:main',
            'apf       = cleaning_robot_algorithms.apf:main',
	        'vacuum    = cleaning_robot_algorithms.vacuum:main',
            'explorer  = cleaning_robot_algorithms.explorer:main',   
            'benchmark = cleaning_robot_algorithms.benchmark:main',
        ],
    },
)
