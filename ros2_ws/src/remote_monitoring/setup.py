from setuptools import setup

package_name = 'remote_monitoring'

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
    description='Remote monitoring module for robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_streaming_node = remote_monitoring.video_streaming_node:main',
            'anomaly_alarm_node = remote_monitoring.anomaly_alarm_node:main',
        ],
    },
)
