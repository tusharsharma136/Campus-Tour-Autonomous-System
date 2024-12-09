from setuptools import setup
import os
from glob import glob

package_name = 'campus_virtual_tour'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'geometry_msgs', 'launch'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Multiagent system for campus virtual tour',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ci_agent = campus_virtual_tour.ci_agent:main',
            'bi_agent = campus_virtual_tour.bi_agent:main',
            'visitor_agent = campus_virtual_tour.visitor_agent:main',
        ],
    },
)
