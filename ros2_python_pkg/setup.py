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
    maintainer='nilutpolk',
    maintainer_email='nkelectronicshlnp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = ros2_python_pkg.publisher:main',
            'subscriber = ros2_python_pkg.subscriber:main',
            'random_num = ros2_python_pkg.random_num:main',
            'odd_even = ros2_python_pkg.odd_even:main',
            'hardware_status = ros2_python_pkg.hardware_status:main',
            'params_use = ros2_python_pkg.params_use:main',
            "add_two_ints_server = ros2_python_pkg.add_two_ints_server:main",
            "add_two_ints_client = ros2_python_pkg.add_two_ints_client:main",
        ],
    },
)
