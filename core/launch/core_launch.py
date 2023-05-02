from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    # Set the paths to the nodes you want to launch
    node_1_package_name = 'core'
    node_1_executable = 'follow_algorithm'

    node_2_package_name = 'qualisys_node'
    node_2_executable = 'qualisys'

    node_3_package_name = 'qtm_logger'
    node_3_executable = 'log'

    # Create the launch description
    ld = LaunchDescription()

    # Add node 1
    node_1 = Node(
        package=node_1_package_name,
        executable=node_1_executable,
        output='screen',
    )
    ld.add_action(node_1)

    # Add node 2
    node_2 = Node(
        package=node_2_package_name,
        executable=node_2_executable,
        output='screen',
    )
    ld.add_action(node_2)

    # Add node 3
    node_3 = Node(
        package=node_3_package_name,
        executable=node_3_executable,
        output='screen',
    )
    ld.add_action(node_3)

    # Create shutdown event handler for qualisys_node
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=node_2,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    ld.add_action(shutdown_on_exit)

    return ld