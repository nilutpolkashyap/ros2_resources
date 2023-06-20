from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_ros2_python_pkg", 
            executable="random_num",
            name="random_num_node",
            parameters=[
            ]
        ),
        Node(
            package="my_ros2_python_pkg", 
            executable="odd_even",
            name="odd_even_node",
            parameters=[
            ]
        ),

    ])