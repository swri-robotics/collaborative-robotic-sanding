import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'crs_utils_py'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[],
    py_modules=[
        'src/joint_state_publisher',
    ],
    # Files we want to install, specifically launch files
#     data_files=[
#         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#         # Include our package.xml file
#         (os.path.join('share', package_name), ['package.xml']),
#         # Include all launch files.
#         (os.path.join('share', package_name, 'src'), glob('*.py'))
#     ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='My awesome package.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'joint_state_publisher = src.joint_state_publisher:main'
        ],
    },
)