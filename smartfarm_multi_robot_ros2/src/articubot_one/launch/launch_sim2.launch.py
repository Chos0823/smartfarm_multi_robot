import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'articubot_one'

    # Launch Arguments
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time')
    use_ros2_control = DeclareLaunchArgument('use_ros2_control', default_value='true', description='Use ros2_control')

    # RSP for both robots
    rsp_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp2.launch.py'
        )]), launch_arguments={'namespace': 'robot1', 'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    rsp_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp2.launch.py'
        )]), launch_arguments={'namespace': 'robot2', 'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Twist Mux for both robots
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    twist_mux_robot1 = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace="robot1",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/robot1/diff_cont/cmd_vel_unstamped')]
    )

    twist_mux_robot2 = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace="robot2",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/robot2/diff_cont/cmd_vel_unstamped')]
    )

    # Gazebo
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Spawn entities for both robots
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot1/robot_description',
            '-entity', 'robot1',
            '-x', '-0.5',
            '-y', '-0.5',
            '-z', '0.0',
        ],
        output='screen'
    )

    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot2/robot_description',
            '-entity', 'robot2',
            '-x', '0.5',
            '-y', '0.5',
            '-z', '0.0',
        ],
        output='screen'
    )

    # Controllers for both robots
    diff_drive_spawner_robot1 = Node(
        package="controller_manager",
        executable="spawner",
        namespace="robot1",
        arguments=["diff_cont"],
    )

    diff_drive_spawner_robot2 = Node(
        package="controller_manager",
        executable="spawner",
        namespace="robot2",
        arguments=["diff_cont"],
    )

    joint_broad_spawner_robot1 = Node(
        package="controller_manager",
        executable="spawner",
        namespace="robot1",
        arguments=["joint_broad"],
    )

    joint_broad_spawner_robot2 = Node(
        package="controller_manager",
        executable="spawner",
        namespace="robot2",
        arguments=["joint_broad"],
    )

    # RViz for both robots
    rviz_config_path_robot1 = os.path.join(get_package_share_directory(package_name), 'config', 'main.rviz')
    rviz_config_path_robot2 = os.path.join(get_package_share_directory(package_name), 'config', 'main2.rviz')

    rviz2_robot1 = Node(
        package='rviz2',
        executable='rviz2',
        namespace="robot1",
        arguments=['-d', rviz_config_path_robot1],
        output='screen'
    )

    rviz2_robot2 = Node(
        package='rviz2',
        executable='rviz2',
        namespace="robot2",
        arguments=['-d', rviz_config_path_robot2],
        output='screen'
    )

    # Launch all nodes
    return LaunchDescription([
        use_sim_time,
        use_ros2_control,
        gazebo,
        rsp_robot1,
        rsp_robot2,
        spawn_robot1,
        spawn_robot2,
        twist_mux_robot1,
        twist_mux_robot2,
        diff_drive_spawner_robot1,
        diff_drive_spawner_robot2,
        joint_broad_spawner_robot1,
        joint_broad_spawner_robot2,
        rviz2_robot1,
        rviz2_robot2
    ])
