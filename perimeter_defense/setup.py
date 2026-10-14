from setuptools import find_packages, setup

package_name = 'perimeter_defense'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/perimeter_defense.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexandru Rizea',
    maintainer_email='alexandrurizea58@gmail.com',
    description='Detection, IFF, tracking and prediction, interceptor allocation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fence_sensor = perimeter_defense.fence_sensor_node:main',
            'person_detector = perimeter_defense.person_detector_node:main',
            'tracker = perimeter_defense.tracker_node:main',
            'iff = perimeter_defense.iff_node:main',
            'interceptor_allocator = perimeter_defense.interceptor_allocator_node:main',
        ],
    },
)
