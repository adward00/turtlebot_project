from setuptools import setup
import os
from glob import glob

package_name = 'dwb_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mg0220',
    maintainer_email='mg0220@todo.todo',
    description='DWB test package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure = dwb_pkg.pure:main',
            'obstacle = dwb_pkg.obstacle:main',
            'rpp_obstacle = dwb_pkg.rpp_obstacle:main',
                
                            ],
    },
)
