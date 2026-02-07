from setuptools import setup

package_name = 'life_assistance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/Reminder.msg']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Life assistance module for robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'schedule_reminder_node = life_assistance.schedule_reminder_node:main',
            'mcp_bridge_node = life_assistance.mcp_bridge_node:main',
        ],
    },
    message_extractors={
        package_name: [
            ('msg/*.msg', 'rosidl_generate_interfaces')
        ]
    },
    buildtool_depend=['rosidl_default_generators'],
    build_depend=['std_msgs'],
    exec_depend=['std_msgs'],
)
