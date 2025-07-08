from setuptools import find_packages, setup

package_name = 'ros2_cv_webcam'

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
    maintainer='haechan',
    maintainer_email='eojin333c@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_subscriber = ros2_cv_webcam.sub:main',
            'img_publisher = ros2_cv_webcam.pub:main',
        ],
    },
)



