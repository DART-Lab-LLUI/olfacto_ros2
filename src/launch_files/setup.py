from setuptools import find_packages, setup

package_name = 'launch_files'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    	('share/launch_files/launch_files', ['launch_files/start_valve_mfc.py']),
            ('share/launch_files/launch_files', ['launch_files/start_olfactometer_vacuum.py']),
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
        ],
    },
)
