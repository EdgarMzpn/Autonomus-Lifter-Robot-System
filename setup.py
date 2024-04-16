from setuptools import find_packages, setup
from glob import glob

package_name = 'puzzlebot_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
       #('share/' + package_name + '/urdf', ['urdf/pendulum.urdf']),
        ('share/' + package_name + '/models', glob('models/*')),
       #('share/' + package_name + '/models', ['models/Arm_2.stl']),
       #('share/' + package_name + '/models', ['models/Base_2.stl']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jesus',
    maintainer_email='jesus@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puzzlebot_kinematics = puzzlebot_challenge.pose_sim:main',
            'odometry = puzzlebot_challenge.localisation:main'
        ],
    },
)
