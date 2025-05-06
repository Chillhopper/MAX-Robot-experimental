import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'my_robot'  # Replace with your package name
    description_folder_name = 'description'
    urdf_file = 'robot.urdf.xacro'  # Replace with your URDF or XACRO filename

    # urdf_path = os.path.join(
    #     get_package_share_directory(pkg_name),
    #     'description',
    #     urdf_file
    # )

    return LaunchDescription([
        # Convert xacro to URDF and publish TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        'xacro ',
                        PathJoinSubstitution([
                            FindPackageShare(pkg_name),
                            description_folder_name,
                            urdf_file
                        ])
                    ]),
                    value_type=str
                )
            }]
        ),

        # Optional: Joint state publisher GUI if you have movable joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Start RViz2 with a blank config or your own .rviz config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory(pkg_name),
                'config',
                'rviz.rviz'  # Replace with your RViz config or remove if not using
            )]
        )
    ])

