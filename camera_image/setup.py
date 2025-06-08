from setuptools import find_packages, setup

package_name = 'camera_image'

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
    description='publishing image from camera intel realsense on non def channel',
    license='GNU-GPL-V3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_image = camera_image.camera_image:main'
        ],
    },
)
