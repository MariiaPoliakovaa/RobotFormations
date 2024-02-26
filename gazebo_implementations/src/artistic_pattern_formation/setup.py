from setuptools import find_packages, setup

package_name = 'artistic_pattern_formation'

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
            'pattern_generation = artistic_pattern_formation.artistic_pattern_controller:main'
        ],
    },
)
