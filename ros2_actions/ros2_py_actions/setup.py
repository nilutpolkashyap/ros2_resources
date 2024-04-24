from setuptools import find_packages, setup

package_name = 'ros2_py_actions'

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
    maintainer='nilutpol',
    maintainer_email='nkelectronicshlnp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_action_server = ros2_py_actions.example_action_server:main',
            'example_action_client = ros2_py_actions.example_action_client:main',
        ],
    },
)
