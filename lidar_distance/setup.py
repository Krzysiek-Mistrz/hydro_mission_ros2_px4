from setuptools import find_packages, setup

package_name = 'lidar_distance'

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
    maintainer='krzychu',
    maintainer_email='krzyskuar@gmail.com',
    description='tf-luna lidar distance reading package',
    license='GNU-GPL-V3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_distance = lidar_distance.lidar_distance:main'
        ],
    },
)
