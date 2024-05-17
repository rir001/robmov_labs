from setuptools import find_packages, setup
from glob import glob

package_name = 'robmov_labs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rir',
    maintainer_email='adrivera1@puc.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_loader            = robmov_labs.pose_loader:main',
            'dead_reckoning         = robmov_labs.dead_reckoning:main',
            'obstacle_detector      = robmov_labs.obstacle_detector:main',
            'pid_angle              = robmov_labs.pid_controller:run_pid_angle',
            'pid_desp               = robmov_labs.pid_controller:run_pid_desp',
        ],
    },
)
