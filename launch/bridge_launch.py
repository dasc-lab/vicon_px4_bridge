from launch import LaunchDescription
from launch_ros.actions import Node


def create_bridge_node(robot_name):

    return Node(
            package='vicon_px4_bridge', executable='bridge', output='screen',
            parameters=[{'px4_name': robot_name, 'vicon_name': robot_name}]
        )

def generate_launch_description():

    hostname = '192.168.1.194' # ip address of vicon computer 
    buffer_size = 200
    topic_namespace = 'vicon'

    robots = ['px4_1']

    bridge_nodes = [create_bridge_node(r) for r in robots]

    vicon_receiver_node = Node(
            package='vicon_receiver', executable='vicon_client', output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}]
        )


    nodes = bridge_nodes + [vicon_receiver_node]

    return LaunchDescription(nodes)

