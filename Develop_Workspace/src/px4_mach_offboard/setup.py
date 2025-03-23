import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'px4_mach_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeyoung',
    maintainer_email='jalim@ethz.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = px4_mach_offboard.offboard_control:main',
                'visualizer = px4_mach_offboard.visualizer:main',
                'velocity_control = px4_mach_offboard.velocity_control:main',
                'control = px4_mach_offboard.control:main',
                'processes = px4_mach_offboard.processes:main',
                'location = px4_mach_offboard.location:main'
                'px4_status = px4_mach_offboard.px4_vehicle_status:main'                
        ],
    },
)
