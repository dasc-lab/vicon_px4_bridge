from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    hostname = '192.168.2.136' #'192.168.1.194'
    buffer_size = 10
    topic_namespace = 'vicon'

    return LaunchDescription([
        Node(
            package='vicon_receiver', executable='vicon_client', output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}]
        ),
        Node(
            package='vicon_px4_bridge', executable='bridge', output='screen',
            parameters=[{'px4_name': 'rover3', 'vicon_name': 'rover3'}]
        )
        Node(
            package='vicon_px4_bridge', executable='bridge', output='screen',
            parameters=[{'px4_name': 'rover2', 'vicon_name': 'rover2'}]
        )
        # Node(
        #     package='vicon_px4_bridge', executable='bridge', output='screen',
        #     parameters=[{'px4_name': 'drone2', 'vicon_name': 'drone2'}]
        # ),
        # Node(
        #     package='vicon_px4_bridge', executable='bridge', output='screen',
        #     parameters=[{'px4_name': 'drone3', 'vicon_name': 'drone3'}]
        # ),
        # Node(
        #     package='vicon_px4_bridge', executable='bridge', output='screen',
        #     parameters=[{'px4_name': 'drone4', 'vicon_name': 'drone4'}]
        # ),
        # Node(
        #     package='vicon_px4_bridge', executable='bridge', output='screen',
        #     parameters=[{'px4_name': 'drone5', 'vicon_name': 'drone5'}]
        # )
        ])
