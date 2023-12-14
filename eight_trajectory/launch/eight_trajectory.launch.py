from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    eight_trajectory = Node(
        package='eight_trajectory',
        executable='eight_trajectory',
        output='screen',
        name='eight_trajectory',
        emulate_tty=True,
    )

    kinematic_model = Node(
        package='kinematic_model',
        executable='kinematic_model',
        output='screen',
        name='kinematic_model',
        emulate_tty=True,
        
    )

    # create and return launch description object
    return LaunchDescription(
        [
            eight_trajectory,
            kinematic_model
        ]
    )
