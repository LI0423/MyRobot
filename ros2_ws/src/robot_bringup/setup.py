from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
        ('share/' + package_name + '/launch', ['launch/emotion_launch.py']),
        ('share/' + package_name + '/launch', ['launch/vision_launch.py']),
        ('share/' + package_name + '/launch', ['launch/xiaozhi.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Robot bringup package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={},
)
