from setuptools import find_packages, setup

package_name = 'mpc_formation_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mariia',
    maintainer_email='s91071@bht-berlin.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'single_robot = mpc_formation_controller.single_robot_controller:main',
            'multi_robot = mpc_formation_controller.multi_robot_controller:main',
        ],
    },
)
