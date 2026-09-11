import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'b3_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Package ament_python : les fichiers launch s'installent ici, comme
        # dans le package bringup d'aeac-2026
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='colin',
    maintainer_email='colinc131@gmail.com',
    description='Nodes fournies pour la formation B3.3 : téléop clavier et décollage automatique',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop = b3_tools.teleop:main',
            'takeoff = b3_tools.takeoff:main',
        ],
    },
)
