from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'my_robot_planar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'meshes'), glob('my_robot_planar/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chiva',
    maintainer_email='chiva@todo.todo',
    description='Muestra un robot en ros2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'real_to_sim = my_robot_planar.real_to_sim:main',
            'sim_to_real = my_robot_planar.sim_to_real:main',
        ],
    },
)