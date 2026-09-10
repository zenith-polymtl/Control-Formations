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
