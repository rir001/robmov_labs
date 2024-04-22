from setuptools import find_packages, setup

package_name = 'robmov_labs'

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
    maintainer='rir',
    maintainer_email='adrivera1@puc.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_reader = robmov_labs.odometry_reader:main',
            'pose_loader = robmov_labs.pose_loader:main',
            'real_pose_reader = robmov_labs.real_pose_reader:main',
            'dead_reckoning = robmov_labs.dead_reckoning:main',
        ],
    },
)
