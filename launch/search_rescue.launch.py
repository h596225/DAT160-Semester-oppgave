import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch nodes for tb3_0
    bug2_node_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='Bug2',
        namespace='tb3_0',
        name='bug2'
    )
    
    robot_handler_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_0',
        name='robot_handler'
    )

    # Launch nodes for tb3_1
    bug2_node_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='Bug2',
        namespace='tb3_1',
        name='bug2'
    )
    
    robot_handler_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_1',
        name='robot_handler'
    )

    # Leader node (no namespace)
    leader_node = Node(
        package='multi_robot_challenge_23',
        executable='leader',
        name='leader'
    )

    # Marker Pose node (using default namespace tb3_5)
    marker_pose_node = Node(
        package='multi_robot_challenge_23',
        executable='marker_pose',
        namespace='tb3_5',
        name='marker_pose'
    )

    # MapFilter node (no specific namespace)
    map_filter_node = Node(
        package='multi_robot_challenge_23',
        executable='map_filter',
        name='map_filter'
    )

    return LaunchDescription([
        bug2_node_tb3_0,
        robot_handler_tb3_0,
        bug2_node_tb3_1,
        robot_handler_tb3_1,
        leader_node,
        marker_pose_node,
        map_filter_node
    ])