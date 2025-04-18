from setuptools import find_packages, setup

package_name = 'sensor_output'

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
    maintainer='olfacto',
    maintainer_email='tom.nielen@hotmail.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = sensor_output.sensor_publisher:main',
            'pid_publisher = sensor_output.pid_publisher:main',
            'sfm_publisher = sensor_output.sfm_publisher:main',
        ],
    },
)
