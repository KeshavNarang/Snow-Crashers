from glob import glob
from setuptools import setup

package_name = 'overwall_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['pad_reference.npy']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='snowcrashers',
    maintainer_email='lukema@stanford.edu',
    description='SnowCrashers test mission: approach a wall with stereo depth, fly over it, land.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'stereo_depth = overwall_nav.stereo_depth_node:main',
            'overwall_mission = overwall_nav.mission_node:main',
            'sensor_check = overwall_nav.sensor_check_node:main',
            'pad_detect = overwall_nav.pad_detect_node:main',
            'remote_link = overwall_nav.remote_link_node:main',
        ],
    },
)
