import launch
import launch_ros.actions

def generate_launch_description():
    waypoints = [
        [0.0, 0.0, 1.0, 0, 0, 0, 1],  # Waypoint 1 (x, y, z, qx, qy, qz, qw)
        [1.0, 0.0, 1.5, 0, 0, 0, 1],  # Waypoint 2
        [1.0, 1.0, 1.0, 0, 0, 0, 1],  # Waypoint 3
        [0.0, 1.0, 1.5, 0, 0, 0, 1]   # Waypoint 4
    ]

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='dummy_waypoint_publisher',
            executable='waypoint_publisher',
            name='waypoint_publisher',
            parameters=[{"waypoints": waypoints}]
        )
    ])
