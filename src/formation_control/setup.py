from setuptools import find_packages, setup
import os 
from glob import glob
package_name = 'formation_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karthik',
    maintainer_email='karthik@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        '2robot = formation_control.2robot:main',
        '3robot = formation_control.3robot:main',
        '3robot2 = formation_control.3robot2:main',
        ],
    },
)
