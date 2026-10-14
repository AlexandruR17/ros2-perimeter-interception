from setuptools import find_packages, setup

package_name = 'perimeter_defense_sim'


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexandru Rizea',
    maintainer_email='alexandrurizea58@gmail.com',
    description='Simulation side of the exercise (common base).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_noise = perimeter_defense_sim.odom_noise_node:main',
            'patrol = perimeter_defense_sim.patrol_node:main',
            'intruder_controller = perimeter_defense_sim.intruder_controller_node:main',
            'truth_markers = perimeter_defense_sim.truth_markers_node:main',
        ],
    },
)
