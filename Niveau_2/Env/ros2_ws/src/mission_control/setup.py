from setuptools import find_packages, setup

package_name = 'mission_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'tf_transformations'],
    zip_safe=True,
    maintainer='Colin Rousseau',
    maintainer_email='colin.rousseau@etud.polymtl.ca',
    description='Répertoire des fichiers qui concerne la génération du ballon à suivre',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balloon = mission_control.ballon_pub:main',
            'monitor = mission_control.monitor:main'
        ],
    },
)
