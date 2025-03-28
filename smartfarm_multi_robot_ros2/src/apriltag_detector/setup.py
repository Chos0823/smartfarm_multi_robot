from setuptools import setup

package_name = 'apriltag_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jmlee',
    maintainer_email='jmlee@example.com',
    description='A ROS 2 package for detecting AprilTags from camera images.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = apriltag_detector.yolo_node:main',
            'smart_farm_node = apriltag_detector.smart_farm:main',
        ],
    },
)

