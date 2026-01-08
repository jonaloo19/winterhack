from glob import glob
from setuptools import setup

package_name = 'winterhack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', '.setup_assistant']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Loo',
    maintainer_email='j.loo@qmul.ac.uk',
    description='Custom Python nodes and utilities for the winterhack workspace.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'locate = winterhack.locate:main',
            'detect = winterhack.detect:main',
            'pick = winterhack.pick:main',
            'drop = winterhack.drop:main',
            'mission_runner = winterhack.mission_runner:main',
        ],
    },
)
