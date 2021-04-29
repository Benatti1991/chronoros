from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    default_vehicle = ["/fullvehiclejson.json"]
    default_lidar = ["/Lidar.json"]
    default_terrain = ["/RigidPlane.json"]
    vehicle_json_path = LaunchConfiguration('vehicle_json_path', default=default_vehicle)
    lidar_json_path = LaunchConfiguration('lidar_json_path', default=default_lidar)
    terrain_json_path = LaunchConfiguration('terrain_json_path', default=default_terrain)
    irr_render = LaunchConfiguration('irr_render', default='true')

    declare_vehicle_json_path = DeclareLaunchArgument(
        'vehicle_json_path',
        default_value=default_vehicle,
        description='Path to vehicle json file')

    declare_lidar_json_path = DeclareLaunchArgument(
        'lidar_json_path',
        default_value=default_lidar,
        description='Path to lidar json description file')

    declare_terrain_json_path = DeclareLaunchArgument(
        'terrain_json_path',
        default_value=default_terrain,
        description='Path to terrain json description file')

    declare_irr_render = DeclareLaunchArgument(
        'irr_render',
        default_value='true',
        description='Enable Irrlicht rendering')




    # Specify the actions
    domain_expert_cmd = Node(
        package='chronoros',
        node_executable='chrono_test_node',
        output='screen',
        parameters=[{'vehicle_json_path': vehicle_json_path},
                    {'lidar_json_path': lidar_json_path},
                    {'terrain_json_path': terrain_json_path},
                    {'irr_render': irr_render}])

    ld = LaunchDescription()
    ld.add_action(declare_vehicle_json_path)
    ld.add_action(declare_lidar_json_path)
    ld.add_action(declare_terrain_json_path)
    ld.add_action(declare_irr_render)
    ld.add_action(domain_expert_cmd)

    return ld