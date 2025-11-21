from setuptools import setup
import os
from glob import glob

package_name = 'my_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ajoute cette ligne pour installer les launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Ajoute cette ligne pour installer les configs
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adminrobot',
    maintainer_email='admin@todo.todo',
    description='My custom nav2 package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
