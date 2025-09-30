import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aan_navigation_clients'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kees van Teeffelen',
    maintainer_email='k.j.vanteeffelen@saxion.nl',
    description='Clients package of smart_diffbot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_client = aan_navigation_clients.docking_client:main',
            'field_cover_client = aan_navigation_clients.field_cover_client:main',
            'row_follow_client = aan_navigation_clients.row_follow_client:main',
            'full_demo = aan_navigation_clients.full_demo:main',
            'supervisory_controldb0 = aan_navigation_clients.supervisory_controldb0:main',
            'supervisory_controldb1 = aan_navigation_clients.supervisory_controldb1:main',
            'test= aan_navigation_clients.test:main',
        ]
    },
)
