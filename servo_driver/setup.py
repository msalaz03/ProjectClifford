from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'servo_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matt',
    maintainer_email='matt@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_pca9685 = pca9685_nodes.servo_pca9685:main',
            'helloworld = pca9685_nodes.helloworld:main',
        ],
    },
)
