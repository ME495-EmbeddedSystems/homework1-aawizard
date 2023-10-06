from setuptools import find_packages, setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/waypoints.launch.xml']),
        ('share/' + package_name, ['config/colors.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aawizard',
    maintainer_email='ananyaagarwal2024@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint = turtle_control.waypoint:main'
        ],
    },
)
