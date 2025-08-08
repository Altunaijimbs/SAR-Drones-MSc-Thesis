from setuptools import setup
import os
from glob import glob

package_name = 'llm_controller'

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
    description='LLM-based drone controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_controller_node = llm_controller.llm_controller_node:main',
            'keep_alive_node = llm_controller.keep_alive_node:main',
            'smart_keep_alive_node = llm_controller.smart_keep_alive_node:main',
            'enhanced_llm = llm_controller.enhanced_llm_controller:main',
            'advanced_llm = llm_controller.advanced_llm_controller:main',
            'practical_llm = llm_controller.practical_llm_controller:main',
            'pattern_llm_bridge = llm_controller.pattern_llm_bridge:main',
        ],
    },
)
