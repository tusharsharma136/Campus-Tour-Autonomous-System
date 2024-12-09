from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='campus_virtual_tour',
            executable='bi_agent',
            name='bi_agent_library',
            parameters=[{'building': 'Library'}]
        ),
        Node(
            package='campus_virtual_tour',
            executable='bi_agent',
            name='bi_agent_lhc',
            parameters=[{'building': 'LHC'}]
        ),
        Node(
            package='campus_virtual_tour',
            executable='bi_agent',
            name='bi_agent_b1_hostel',
            parameters=[{'building': 'B1 Hostel'}]
        ),
        Node(
            package='campus_virtual_tour',
            executable='ci_agent',
            name='ci_agent'
        ),
        Node(
            package='campus_virtual_tour',
            executable='visitor_agent',
            name='visitor_1'
        ),
        Node(
            package='campus_virtual_tour',
            executable='visitor_agent',
            name='visitor_2'
        )
    ])

