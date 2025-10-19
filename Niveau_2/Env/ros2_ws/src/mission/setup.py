from setuptools import find_packages, setup

package_name = 'mission'

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
    maintainer='Colin Rousseau',
    maintainer_email='colin.rousseau@etud.polymtl.ca',
    description='Répertoire des fichiers qui concerne la mission principale',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #nom_choisi = package.nom_du_fichier_sans_extension : main
            'exemple = mission.exemple:main'
        ],
    },
)
