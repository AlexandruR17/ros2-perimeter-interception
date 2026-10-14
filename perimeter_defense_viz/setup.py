from setuptools import find_packages, setup

package_name = 'perimeter_defense_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/viz.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexandru Rizea',
    maintainer_email='alexandrurizea58@gmail.com',
    description='Tactical RViz visualisation of the base layout, the UGV fleet and the status board.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'layout_markers = perimeter_defense_viz.layout_markers_node:main',
            'fleet_markers = perimeter_defense_viz.fleet_markers_node:main',
            'status_board = perimeter_defense_viz.status_board_node:main',
        ],
    },
)
