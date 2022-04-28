mport os
from glob import glob
from setuptools import setup

package_name = 'lego_epike_interface'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Chris Iverach-Brereton',
    author_email='civerachb@clearpathrobotics.com',
    maintainer='Chris Iverach-Brereton',
    maintainer_email='civerachb@clearpathrobotics.com',
    keywords=['lego', 'mindstorms'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 interface for using the Lego Mindstorms/Lego Spike Prime hub',
    license='BSD',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'lego_spike_interface': [
            'serial_interface = serial_interface_node:main'
        ],
    },
)