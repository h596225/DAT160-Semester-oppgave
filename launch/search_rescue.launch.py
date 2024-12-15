import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch nodes for tb3_0
    bug2_node_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='bug2',
        namespace='tb3_0',
        name='bug2',
        output='screen',
        arguments=['tb3_0', 'right']
    )

    robot_handler_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_0',
        name='robot_handler',
        output='screen',
        parameters=[{'namespace': 'tb3_0'}]
    )

    # Launch nodes for tb3_1
    bug2_node_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='bug2',
        namespace='tb3_1',
        name='bug2',
        output='screen',
        arguments=['tb3_1', 'left'] 
       )   

    robot_handler_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_1',
        name='robot_handler',
        output='screen',
        parameters=[{'namespace': 'tb3_1'}]
    )

    # Leader node (no namespace)
    leader_node = Node(
        package='multi_robot_challenge_23',
        executable='leader',
        name='leader',
        output='screen'
    )

    return LaunchDescription([
        bug2_node_tb3_0,
        robot_handler_tb3_0,
        bug2_node_tb3_1,
        robot_handler_tb3_1,
        leader_node,
    ])
