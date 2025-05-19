from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'golf_cart'

def package_files(directory):
    paths = []
    for root, _, filenames in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(root, filename)
            install_path = os.path.join('share', package_name, root)
            paths.append((install_path, [file_path]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *package_files('config'),
        *package_files('launch'),
        *package_files('meshes'),
        *package_files('rviz'),
        *package_files('urdf'),
        *package_files('worlds'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanjana',
    maintainer_email='sanjana@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_human = golf_cart.detect_human:main'
        ],
    },
)