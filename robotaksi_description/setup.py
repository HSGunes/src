from setuptools import setup
import os
from glob import glob

package_name = 'robotaksi_description'

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                if path not in paths_dict:
                    paths_dict[path] = []
                paths_dict[path].append(os.path.join(path, filename))

    for path in paths_dict:
        data_files.append((os.path.join('share', package_name, path), paths_dict[path]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files([
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**/*', recursive=True)),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ], ['models']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)