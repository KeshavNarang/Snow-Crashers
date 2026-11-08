from glob import glob
from setuptools import setup

package_name = 'manual_flight'

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
    maintainer_email='odyssey@stanford.edu',
    description='Hand-flown flight, fully recorded; runs no flight_interface or mission.',
    license='Apache-2.0',
)
