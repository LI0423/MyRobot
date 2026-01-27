from setuptools import setup

package_name = 'vision_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Vision perception module for robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detection_node = vision_perception.face_detection_node:main',
            'object_detection_node = vision_perception.object_detection_node:main',
            'scene_understanding_node = vision_perception.scene_understanding_node:main',
        ],
    },
)
