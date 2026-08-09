from glob import glob

from setuptools import find_packages, setup

package_name = 'xgo_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/cartographer', glob('config/cartographer/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tamme',
    maintainer_email='17968111+luckyTamme@users.noreply.github.com',
    description=(
        "Localisation for the XGO2 quadruped: the robot's pose relative to "
        'where it started, with a selectable SLAM backend.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'localization_node = xgo_localization.localization_node:main',
        ],
    },
)
