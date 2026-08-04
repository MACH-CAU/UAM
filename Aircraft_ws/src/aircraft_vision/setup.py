from setuptools import find_packages, setup

package_name = 'aircraft_vision'

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
    maintainer='yerin',
    maintainer_email='nayl0301@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gate_detector_node = aircraft_vision.gate_detector_node:main',
            'gate_decision_node = aircraft_vision.gate_decision_node:main',
        ],
    },
)
