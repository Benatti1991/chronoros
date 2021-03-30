from launch import LaunchDescription
import launch_ros.actions



def generate_launch_description():
#    clp = get_args()
    return LaunchDescription([
        launch_ros.actions.Node(
            package='chronoros',
            node_executable='chrono_test_node',
            output='screen',
            parameters=[
                {"my_parameter": "chimichanga"}
            ]),
    ])