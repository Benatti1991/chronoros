from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    vals = ['chimichanga']
    my_parameter = LaunchConfiguration('my_parameter', default=vals)

    declare_my_parameter_cmd = DeclareLaunchArgument(
        'model_files',
        default_value=vals,
        description='PDDL Model file')

    # Specify the actions
    domain_expert_cmd = Node(
        package='chronoros',
        node_executable='chrono_test_node',
        output='screen',
        parameters=[{'my_parameter': my_parameter}])

    ld = LaunchDescription()
    ld.add_action(declare_my_parameter_cmd)
    ld.add_action(domain_expert_cmd)

    return ld