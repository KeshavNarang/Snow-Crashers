from glob import glob

from setuptools import find_packages, setup

setup(
    name='drone_ref', version='0.2.0', packages=find_packages(include=['drone_ref', 'drone_ref.*']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/drone_ref']),
                ('share/drone_ref', ['package.xml']),
                ('share/drone_ref/config', glob('config/*.yaml') + glob('config/*.json')),
                ('share/drone_ref/assets', ['assets/course-v3.obj', 'assets/pad_reference.npy']),
                ('share/drone_ref/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'],
    python_requires='>=3.10', zip_safe=False,
    maintainer='SnowCrashers', maintainer_email='snowcrashers@example.invalid',
    description='Goal navigation on a live map for the DroneHacks course (Sky Beta)', license='MIT',
    entry_points={'console_scripts': ['goalnav_node=drone_ref.goalnav_node:main',
                                    'stereo_node=drone_ref.stereo_node:main']},
)
