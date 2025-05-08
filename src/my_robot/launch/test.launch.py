import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'my_robot'  # Replace with your package name
    description_folder_name = 'description'
    urdf_file = 'main.urdf.xacro'  # Replace with your URDF or XACRO filename

    #---------------------GAZEBO CLASSIC

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        # launch_arguments={'pause': 'true'}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.5'  # Small z offset avoids clipping through ground
        ],
        output='screen'
    )

    #---------------------GAZEBO IGNITION

    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('ros_ign_gazebo'),
    #             'launch',
    #             'ign_gazebo.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={'ign_args': '-r -v 4 empty.sdf'}.items()
    # )

    # spawn_entity = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'run', 'ros_ign_gazebo', 'create',
    #         '-name', 'my_robot',
    #         '-topic', 'robot_description'
    #     ],
    #     output='screen'
    # )



    return LaunchDescription([
        # Convert xacro to URDF and publish TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
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
            output='screen',
            parameters=[{'use_sim_time' : True}]
        ),

        # Start RViz2 with a blank config or your own .rviz config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time' : True}],
            arguments=['-d', os.path.join(
                get_package_share_directory(pkg_name),
                'config',
                'rviz_odom.rviz'  # Replace with your RViz config or remove if not using
            )]
        ),

        gazebo_launch, spawn_entity



    ])

