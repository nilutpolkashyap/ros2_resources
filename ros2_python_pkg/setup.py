from setuptools import setup

package_name = 'ros2_python_pkg'

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
    maintainer='nilutpolkashyap',
    maintainer_email='nilutpolkashyap@gmail.com',
    description='ROS 2 Python code examples package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = ros2_python_pkg.publisher:main',
            'subscriber = ros2_python_pkg.subscriber:main',
            'random_num = ros2_python_pkg.random_num:main',
            'odd_even = ros2_python_pkg.odd_even:main',
        ],
    },
)
