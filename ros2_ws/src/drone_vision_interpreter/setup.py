from setuptools import setup
import os
from glob import glob

package_name = 'drone_vision_interpreter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Vision to text conversion for LLM-controlled drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_to_text_node = drone_vision_interpreter.vision_to_text_node:main',
            'object_detector_node = drone_vision_interpreter.object_detector_node:main',
        ],
    },
)
