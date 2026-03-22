from setuptools import find_packages, setup

package_name = 'tauv_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'wrench = tauv_autonomy.wrench:main', 
            'force_to_gain = tauv_autonomy.force_to_gain:main',
            'controller = tauv_autonomy.controller:main',
        ],
    },
)