from setuptools import setup

package_name = 'ros2_python_lifecycle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nilutpol Kashyap',
    maintainer_email='nilutpolkashyap@todo.todo',
    description='ROS 2 Python Lifecycle Nodes Package',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifecycle_node = ros2_python_lifecycle.lifecycle_node:main',
        ],
    },
)
