import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    mode = LaunchConfiguration('mode')  # 추가: mapping/localization 선택
    default_params_file = os.path.join(get_package_share_directory("articubot_one"),
                                       'config', 'mapper_params_online_async.yaml')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # 추가: mode 설정 (mapping 또는 localization)
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: "mapping" (build a new map) or "localization" (use existing map)')

    has_node_params = HasNodeParams(source_file=params_file, node_name='slam_toolbox')

    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])

    log_param_change = LogInfo(msg=['provided params_file ', params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file],
                               condition=UnlessCondition(has_node_params))

    # Mapping 모드 (새로운 맵 생성)
    start_mapping_slam_toolbox_node = Node(
        parameters=[
          actual_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(PythonExpression(['"', mode, '" == "mapping"']))
    )

    # Localization 모드 (기존 맵 사용)
    start_localization_slam_toolbox_node = Node(
        parameters=[
          actual_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(PythonExpression(['"', mode, '" == "localization"']))
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mode_cmd)  # mode 추가
    ld.add_action(log_param_change)
    ld.add_action(start_mapping_slam_toolbox_node)
    ld.add_action(start_localization_slam_toolbox_node)

    return ld
