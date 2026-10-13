import os

from setuptools import find_packages, setup

package_name = 'perimeter_defense_description'


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
for d in ['urdf', 'config']:
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
    description='LUP UGV robot description.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
