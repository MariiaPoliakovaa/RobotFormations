from setuptools import find_packages, setup

package_name = 'target_assignement_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'turtlebot_controller'
    ],
    zip_safe=True,
    maintainer='mariia',
    maintainer_email='s91071@bht-berlin.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_assignement = target_assignement_controller.target_assignement_controller:main',
        ],
    },
)
