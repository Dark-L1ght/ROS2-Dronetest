from setuptools import find_packages, setup

package_name = 'dronetest'

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
    maintainer='Abiyyu Nizar',
    maintainer_email='abiyyunizar@gmail.com',
    description='Drone Object Detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_processor = dronetest.camera_processor:main',
            'object_detector = dronetest.object_detector:main',
            'drone_controller = dronetest.drone_controller:main',
            'mission_manager = dronetest.mission_manager:main',
            'movement_test = dronetest.movement_test:main'
        ],
    },
)
