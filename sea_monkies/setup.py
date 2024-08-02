from setuptools import find_packages, setup

package_name = 'sea_monkies'

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
    maintainer='yafeekhan',
    maintainer_email='yafee22.khan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement = sea_monkies.movement:main',
            'arming = sea_monkies.arming:main',
            'pid_controller = sea_monkies.pid_controller:main',
            'pressure = sea_monkies.pressure:main',
            'angle_controller = sea_monkies.angle_controller:main',
            'cam = sea_monkies.live_apriltag:main',
            'lights = sea_monkies.lights:main',
            'timer = sea_monkies.timer:main',
            'molo = sea_monkies.motion_lotion:main',
            'integration = sea_monkies.integration_fucking_hell:main'
        ],
    },
)
