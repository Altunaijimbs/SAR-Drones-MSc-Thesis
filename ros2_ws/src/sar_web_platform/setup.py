from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sar_web_platform'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include templates and static files
        ('share/' + package_name + '/templates', glob('templates/*.html')),
        ('share/' + package_name + '/static/css', glob('static/css/*.css')),
        ('share/' + package_name + '/static/js', glob('static/js/*.js')),
        # Include launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Install executable to lib/package_name for ROS2
        ('lib/' + package_name, ['scripts/web_server']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SAR Team',
    maintainer_email='sar@example.com',
    description='Web-based control platform for SAR drone system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = sar_web_platform.web_server:main',
        ],
    },
)