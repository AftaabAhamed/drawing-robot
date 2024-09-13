from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hb_task4b',
            namespace='controller',
            executable='controller_bot1',
            name='controller1'
        ),
        Node(
            package='hb_task4b',
            namespace='controller',
            executable='controller_bot2',
            name='controller2'
        ),
        Node(
            package='hb_task4b',
            namespace='controller',
            executable='controller_bot3',
            name='controller3',
        ),
        Node(
            package='hb_task4b',
            namespace='cam',
            executable='visualisation',
            name='cam visualize',
        ),
        Node(
           package='hb_task4b',
           namespace='draw',
           executable='publish_draw_simple',
           name='simple_draw_pub',
        ),
    ])