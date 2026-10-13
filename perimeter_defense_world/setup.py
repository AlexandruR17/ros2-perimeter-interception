import os

from setuptools import find_packages, setup

package_name = 'perimeter_defense_world'


def tree(src: str):
    """Install a directory tree under share/<package>/<src>, keeping the layout."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            dest = os.path.join('share', package_name, root)
            out.append((dest, [os.path.join(root, f) for f in files]))
    return out


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
for d in ['config', 'worlds', 'maps', 'models']:
    data_files += tree(d)

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexandru Rizea',
    maintainer_email='alexandrurizea58@gmail.com',
    description='FOB-3 base world, map and assets.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'generate_world = perimeter_defense_world.generate_world:main',
            'generate_map = perimeter_defense_world.generate_map:main',
            'render_ortho = perimeter_defense_world.render_ortho:main',
            'check_actor_lidar = perimeter_defense_world.check_actor_lidar:main',
        ],
    },
)
