from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name + '/launch',
            ['launch/navigation.launch.py', 'launch/navigation_server.launch.py',
             'launch/nav2.launch.py']),
        ('share/' + package_name + '/params',
            ['params/navigation_server.yaml', 'params/nav2_params.yaml',
             'params/nav2_params_default.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='douglas',
    maintainer_email='arthur.guihaire@gmail.com',
    description='Navigation server, client, and Nav2 integration for the AUV.',
    license='GPLv3',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation_server = navigation.navigation_server:main',
        ],
    },
)
