from setuptools import find_packages, setup
import os
from glob import glob 

package_name = 'four_wheels_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'),glob('urdf/*')),
        (os.path.join('share',package_name,'config'),glob('config/*')),
        (os.path.join('share',package_name,'map'),glob('map/*')),
        (os.path.join('share', package_name,'world'), glob("world/*")),
        (os.path.join('share', package_name,'AI_model'), glob('AI_model/*')),
    ],
    package_data={
            package_name: ['AI_model/*.pt'],
        },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mark',
    maintainer_email='kimutai.workspace@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "ai=four_wheels_robot.image_processing:main",
            "bb=four_wheels_robot.AI_blackboard:main",
        ],
    },
)

# four_wheels_robot/AI_blackboard.py