from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lerobot_101_6_salut_play'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'moveit_overlay', 'config'),
         glob('moveit_overlay/config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='potato',
    maintainer_email='potato@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
