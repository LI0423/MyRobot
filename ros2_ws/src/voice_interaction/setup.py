from setuptools import setup
from glob import glob
import os

package_name = 'voice_interaction'

def get_data_files():
    data_files = []
    data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
    data_files.append(('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))))
    data_files.append(('share/' + package_name, ['package.xml']))
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Voice interaction package for XiaoPei robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xiaozhi_assistant_node = voice_interaction.xiaozhi_assistant_node:main',
        ],
    },
)