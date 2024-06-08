from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'teleop_controller'
#hello 
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matt',
    maintainer_email='msalaz03@uoguelph.ca',
    description='Joystick Controller + IK',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'CliffordJoy = scripts.CliffordJoy:main',
            'CliffordTF = scripts.clifford_tf:main',
        ],
    },
)
