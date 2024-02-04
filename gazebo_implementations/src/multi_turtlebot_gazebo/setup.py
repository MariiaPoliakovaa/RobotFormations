import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'multi_turtlebot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'models/turtlebot3_burger'), glob('models/turtlebot3_burger/*')),
    ],
    install_requires=[
        'setuptools',
        'launch'
    ],
    zip_safe=True,
    maintainer='mariia',
    maintainer_email='s91071@bht-berlin.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
)
