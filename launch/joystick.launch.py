import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "gazebo_ackermann_steering_vehicle"

    vehicle_params_path = os.path.join(get_package_share_directory(package_name),
                                       'config', 'parameters.yaml')
    
    joy_node = Node(package="joy", executable="joy_node")
    
    joystick_controller_node = Node(package=package_name,
                                    executable='joystick_controller',
                                    parameters=[vehicle_params_path],
                                    output='screen')

    return LaunchDescription([joy_node,
                              joystick_controller_node])
