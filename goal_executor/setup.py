from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'goal_executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoc3hc',
    maintainer_email='chanhhoang999x@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_executor = goal_executor.goal_executor:main',
            'test_goal_executor = goal_executor.test_goal_executor:main'
        ],
    },
)
