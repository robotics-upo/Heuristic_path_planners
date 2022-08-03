from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/'),
        DeclareLaunchArgument('map_name', default_value='mbzirc_challenge3'),
        DeclareLaunchArgument('map', default_value='$(find heuristic_planners)/resources/3dmaps/$(arg map_name).bt'),
        DeclareLaunchArgument('algorithm_name', default_value='costlazythetastar'),
        DeclareLaunchArgument('world_size_x', default_value='60'),
        DeclareLaunchArgument('world_size_y', default_value='60'),
        DeclareLaunchArgument('world_size_z', default_value='20'),
        DeclareLaunchArgument('resolution', default_value='0.2'),
        DeclareLaunchArgument('inflate_map', default_value='true'),
        DeclareLaunchArgument('inflation_size', default_value='$(arg resolution)'),
        DeclareLaunchArgument('heuristic', default_value='euclidean'),
        DeclareLaunchArgument('save_data', default_value='false'),
        DeclareLaunchArgument('data_folder', default_value='$(env HOME)'),
        DeclareLaunchArgument('overlay_markers', default_value='false'),
        DeclareLaunchArgument('cost_weight', default_value='4.0'),
        DeclareLaunchArgument('max_line_of_sight_distance', default_value='2.0'),
        DeclareLaunchArgument('cost_scaling_factor', default_value='2.0'),
        DeclareLaunchArgument('robot_radius', default_value='0.4'),
        Node(
            package='heuristic_planners',
            executable='planner_ros_node',
            namespace=LaunchConfiguration('namespace'),
            remappings=[ ('points','/grid3d/map_point_cloud') ],
            parameters=[ {'name': 'map_path', 'value': LaunchConfiguration('map')},
                        {'name': 'world_size_x', 'value': LaunchConfiguration('world_size_x')},
                        {'name': 'world_size_y', 'value': LaunchConfiguration('world_size_y')},
                        {'name': 'world_size_z', 'value': LaunchConfiguration('world_size_z')},
                        {'name': 'resolution', 'value': LaunchConfiguration('resolution')},
                        {'name': 'inflate_map', 'value': LaunchConfiguration('inflate_map')},
                        {'name': 'inflation_size', 'value': LaunchConfiguration('inflation_size')},
                        {'name': 'save_data_file', 'value': LaunchConfiguration('save_data')},
                        {'name': 'overlay_markers', 'value': LaunchConfiguration('overlay_markers')},
                        {'name': 'data_folder', 'value': LaunchConfiguration('data_folder')},
                        {'name': 'algorithm', 'value': LaunchConfiguration('algorithm_name')},
                        {'name': 'cost_weight', 'value': LaunchConfiguration('cost_weight')},
                        {'name': 'max_line_of_sight_distance', 'value': LaunchConfiguration('max_line_of_sight_distance')},
                        {'name': 'heuristic', 'value': LaunchConfiguration('heuristic')},
                        {'name': 'cost_scaling_factor', 'value': LaunchConfiguration('cost_scaling_factor')},
                        {'name': 'robot_radius', 'value': LaunchConfiguration('robot_radius')}
                        ],
            output='screen',
            emulate_tty=True
        )
    ])


