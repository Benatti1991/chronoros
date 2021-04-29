from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    default_vehicle = ["/fullvehiclejson.json"]
    vehicle_json_path = LaunchConfiguration('vehicle_json_path', default=default_vehicle)

    declare_vehicle_json_path = DeclareLaunchArgument(
        'model_files',
        default_value=default_vehicle,
        description='PDDL Model file')

    # Specify the actions
    domain_expert_cmd = Node(
        package='chronoros',
        node_executable='chrono_test_node',
        output='screen',
        parameters=[{'vehicle_json_path': vehicle_json_path}])

    ld = LaunchDescription()
    ld.add_action(declare_vehicle_json_path)
    ld.add_action(domain_expert_cmd)

    return ld