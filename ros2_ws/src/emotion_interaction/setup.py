from setuptools import setup
import os
from glob import glob

package_name = 'emotion_interaction'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='The emotion_interaction package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emotion_engine_node = emotion_interaction.emotion_engine_node:main',
            'voice_recognition_node = emotion_interaction.voice_recognition_node:main',
            'text_to_speech_node = emotion_interaction.text_to_speech_node:main',
        ],
    },
)
