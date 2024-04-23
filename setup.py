from setuptools import find_packages, setup

package_name = 'ros2_kot_prog'

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
    maintainer='blengyel',
    maintainer_email='lengyel.balazs.t@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'psm_grasp = ros2_kot_prog.psm_grasp:main',
            'dummy_marker = ros2_kot_prog.dummy_marker:main',
            'interactive_marker = ros2_kot_prog.interactive_marker:main',
            'marker_factory = ros2_kot_prog.marker_factory:main'
        ],
    },
)
