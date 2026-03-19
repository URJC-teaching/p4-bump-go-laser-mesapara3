
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bumpgo_cmd = Node(
            package='fsm_bumpgo',
            executable='laser_go_node',
            name='lasergo_laser_node',
            output='screen',
            remappings=[
                ('input_laser', '/scan_filtered'), #/scan_filtered , _raw
                ('/out_vel', '/cmd_vel'),
            ]
        )

    ld = LaunchDescription()
    ld.add_action(bumpgo_cmd)

    return ld
