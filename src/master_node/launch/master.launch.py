from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    master_node = Node(
        package = 'master_node',
        executable = 'master_node',
        output = 'screen'
        
    )
    '''
    injection_node = Node(
        package = 'injection',
        executable = 'injection_node',
        output = 'screen'
    )
    '''
    move_node = Node(
        package = 'move_node',
        executable = 'move_node',
        output = 'screen'
        
    )
    slam_node_node = Node(
        package = 'slam_node',
        executable = 'slam_node',
        output = 'screen'
    )
    simomuni4_node = Node(
        package= 'simomuni4_node',
        executable='simomuni4_node',
        output='screen'
    )
    
    ld.add_action(master_node)
    #ld.add_action(move_node)
    ld.add_action(slam_node_node)
    ld.add_action(simomuni4_node)
    
    return ld