from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'Mini_Dog_URDF_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_sag_node = Mini_Dog_URDF_description.battery_sag_node:main',
            'open_loop_visualizer_node = Mini_Dog_URDF_description.open_loop_visualizer_node:main',
            'stand_straight_node = Mini_Dog_URDF_description.kinematics.stand_straight:main',
            'walk_forward_node = Mini_Dog_URDF_description.kinematics.walk_forward:main',
            'ratchet_controller_node = Mini_Dog_URDF_description.ratchet_controller:main',
            'skateboard_forward_node = Mini_Dog_URDF_description.kinematics.skateboard_forward:main',
            'skateboard_sim_node = Mini_Dog_URDF_description.kinematics.skateboard_sim:main',
            'crawl_gait_node = Mini_Dog_URDF_description.kinematics.crawl_gait:main',
            'master_teleop_node = Mini_Dog_URDF_description.kinematics.master_teleop:main',
            'pivot_turn_node = Mini_Dog_URDF_description.kinematics.pivot_turn:main',
            'wheelie_glide_node = Mini_Dog_URDF_description.kinematics.wheelie_glide:main',
        ],
    },
)

