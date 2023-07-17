from launch import LaunchDescription
import launch_ros.actions 

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='map_localization',
            executable='map_localization',
            output='screen'
            ),
        ])

