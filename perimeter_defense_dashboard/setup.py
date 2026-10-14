import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'perimeter_defense_dashboard'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
]
# The web application keeps its directory layout.
for root, _, files in os.walk('web'):
    if files:
        data_files.append((os.path.join('share', package_name, root),
                           [os.path.join(root, f) for f in files]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexandru Rizea',
    maintainer_email='alexandrurizea58@gmail.com',
    description='Web C2 dashboard (rosbridge, Leaflet, MIL-STD-2525 symbols, MGRS grid).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'c2_state = perimeter_defense_dashboard.c2_state_node:main',
            'dashboard_server = perimeter_defense_dashboard.server_node:main',
        ],
    },
)
