from glob import glob
from setuptools import setup

package_name = 'bringup'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # <-- this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='colin',
    description='bringup',
)
