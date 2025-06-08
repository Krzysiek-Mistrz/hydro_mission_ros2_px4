from setuptools import find_packages, setup

package_name = 'hydrolab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #dodanie launcha startujacego wszystkie nody + dodatki
        ('share/' + package_name + '/launch', ['launch/hydrolab_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krzychu',
    maintainer_email='krzyskuar@gmail.com',
    description='drone controller (rusz z p.A do p.B w p.B wykryj aruko obniz sie o xm wroc do p.B i wroc do p.A)',
    license='Bruh... xD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller = hydrolab.drone_controller:main',
            'pool_tracker = hydrolab.pool_tracker:main',
        ],
    },
)
