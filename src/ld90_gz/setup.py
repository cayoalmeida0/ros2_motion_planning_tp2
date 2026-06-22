from setuptools import setup
from glob import glob
import os

package_name = 'ld90_gz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models/2m_wall'), glob('obstacles/2m_wall/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cayo-sousa',
    maintainer_email='cayo-sousa@todo.todo',
    description='Simulação do Omron LD90 no Gazebo Harmonic com ROS 2 Jazzy.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_map = ld90_gz.grid_map:main',
            'astar_planner = ld90_gz.astar_planner:main',
            'gvd_planner = ld90_gz.brushfire_gvd:main',
            'path_follower = ld90_gz.path_follower:main',
        ],
    },
)