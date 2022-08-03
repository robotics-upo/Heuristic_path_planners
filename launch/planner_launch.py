from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    map_default = PathJoinSubstitution([
        FindPackageShare('heuristic_planners'),
        'resources', '3dmaps', 'mbzirc_challenge3.bt'
    ])
#
#    map_default_ = PathJoinSubstitution([
 #       FindPackageShare('heuristic_planners'),
#        'resources', '3dmaps', LaunchConfiguration('map_name')+'.bt'
#    ])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/'),
        DeclareLaunchArgument('map_name', default_value='mbzirc_challenge3'),
        #DeclareLaunchArgument('map_name', default_value='$(find heuristic_planners)/resources/3dmaps/$(arg map_name).bt'),
        DeclareLaunchArgument('map', default_value=map_default),
        DeclareLaunchArgument('algorithm_name', default_value='costlazythetastar'),
        DeclareLaunchArgument('world_size_x', default_value='60.0'),
        DeclareLaunchArgument('world_size_y', default_value='60.0'),
        DeclareLaunchArgument('world_size_z', default_value='20.0'),
        DeclareLaunchArgument('resolution', default_value='0.2'),
        DeclareLaunchArgument('inflate_map', default_value='true'),
        DeclareLaunchArgument('inflation_size', default_value=LaunchConfiguration('resolution')),
        DeclareLaunchArgument('heuristic', default_value='euclidean'),
        DeclareLaunchArgument('save_data', default_value='false'),
        DeclareLaunchArgument('data_folder', default_value=EnvironmentVariable('HOME')),
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
            parameters=[{'map_path': LaunchConfiguration('map'),
                        'world_size_x': LaunchConfiguration('world_size_x'),
                        'world_size_y': LaunchConfiguration('world_size_y'),
                        'world_size_z': LaunchConfiguration('world_size_z'),
                        'resolution': LaunchConfiguration('resolution'),
                        'inflate_map': LaunchConfiguration('inflate_map'),
                        'inflation_size': LaunchConfiguration('inflation_size'),
                        'save_data_file': LaunchConfiguration('save_data'),
                        'overlay_markers': LaunchConfiguration('overlay_markers'),
                        'data_folder': LaunchConfiguration('data_folder'),
                        'algorithm': LaunchConfiguration('algorithm_name'),
                        'cost_weight': LaunchConfiguration('cost_weight'),
                        'max_line_of_sight_distance': LaunchConfiguration('max_line_of_sight_distance'),
                        'heuristic': LaunchConfiguration('heuristic'),
                        'cost_scaling_factor': LaunchConfiguration('cost_scaling_factor'),
                        'robot_radius': LaunchConfiguration('robot_radius')
                        }],
            output='screen',
            emulate_tty=True
        )
        #Node(
        #    package='rviz2',
        #    namespace='',
        #    executable='rviz2',
        #    name='rviz2'
        #)
    ])
    
