# for articubot_one robot
# 이 명령어들 대신 런치파일로 사용하는것을 추천
# 모든 명령어는 articubot에 있는 워크스페이스에서 실행한다


# (1) SLAM


# 미리 지정되어있는 world 파일을 가제보로 불러온다(launch_sim.launch.py 파일 및 world 경로 확인)
ros2 launch articubot_one launch_sim.launch.py world:=$HOME/b4_ws/src/articubot_one/worlds/obstacles.world

# main.rviz를 통한 rviz 시각화(launch_sim.launch.py에 이미 포함되어 있음)
rviz2 -d $HOME/b4_ws/src/articubot_one/config/main.rviz

# SLAM (mapper_params_online_async.yaml에 mapping 모드와 localization 모드 존재, 맵핑 시 mapping 모드로 변경 필수)
ros2 launch slam_toolbox online_async_launch.py params_file:=$HOME/b4_ws/src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=true mode:=localization

or

ros2 launch articubot_one online_async_launch.py mode:=localization

#맵 저장 형식 (rviz plugin 사용)
my_map_save.yaml 
my_map_serial.data

# 로봇 컨트롤러
ros2 run turtlebot3_teleop teleop_keyboard




# (2) 맵 로드 및 2D pose명령어

# my_map_save.yaml 파일을 nav2 맵에 저장하기 위한 요청
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$HOME/b4_ws/my_map_save.yaml -p use_sim_time:=true

# lifecycle_bringup을 통해 nav2 맵에 저장
ros2 run nav2_util lifecycle_bringup map_server

# 2D estimate pose를 위한 요청
ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true

# lifecycle_bringup을 통해 2D estimate pose 준비
ros2 run nav2_util lifecycle_bringup amcl

#간단 맵로드  localization 런치파일
ros2 launch articubot_one localization_launch.py






# (3) nav2 관련명령어

# twist_mux
ros2 run twist_mux twist_mux --ros-args --params-file $HOME/b4_ws/src/articubot_one/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped

# 미리 지정되어있는 world 파일을 가제보로 불러온다(launch_sim.launch.py 파일 및 world 경로 확인)
ros2 launch articubot_one launch_sim.launch.py world:=$HOME/b4_ws/src/articubot_one/worlds/obstacles.world

# SLAM (mapper_params_online_async.yaml에 mapping 모드와 localization 모드 존재, 맵핑 시 mapping 모드로 변경 필수)
ros2 launch slam_toolbox online_async_launch.py params_file:=$HOME/b4_ws/src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=true

# nav2 with costmap
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

# nav2 with amcl
ros2 launch nav2_bringup localization_launch.py map:=$HOME/b4_ws/my_map_save.yaml use_sim_time:=true map_subscribe_transient_local:=true

# 간단 nav2 런치파일
ros2 launch articubot_one localization_launch.py
ros2 launch articubot_one navigation_launch.py
