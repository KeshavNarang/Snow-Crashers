from glob import glob
from setuptools import setup

package_name = 'snowcrashers_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='snowcrashers',
    maintainer_email='lukema@stanford.edu',
    description='SnowCrashers skybeta mission: up 2 m, forward 2 m, land.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'up_forward_land = snowcrashers_nav.up_forward_land:main',
        ],
    },
)
