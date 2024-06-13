import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arduino_ip',
            default_value='192.168.1.120',
            description='IP address of the Arduino'
        ),
        DeclareLaunchArgument(
            'modbus_port',
            default_value='502',
            description='Modbus TCP port'
        ),
        Node(
            package='recieve_ultra_sonic',
            executable='modbus_bridge.py',
            name='modbus_bridge',
            parameters=[{
                'arduino_ip': LaunchConfiguration('arduino_ip'),
                'modbus_port': LaunchConfiguration('modbus_port'),
            }]
        )
    ])
