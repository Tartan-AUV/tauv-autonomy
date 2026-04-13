from setuptools import find_packages, setup

package_name = 'tauv_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/motor_equations.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'thruster_forces = tauv_controller.thruster_forces:main', 
            'controller = tauv_controller.controller:main',
            'oscillating = tauv_controller.oscintillating:main',
            'thruster_rpms = tauv_controller.thruster_rpms:main',

        ],
    },
)