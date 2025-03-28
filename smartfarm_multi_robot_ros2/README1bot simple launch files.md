# 미리 지정되어있는 world 파일을 가제보로 불러온다(launch_sim.launch.py 파일 및 world 경로 확인)
ros2 launch articubot_one launch_sim.launch.py world:=$HOME/b4_ws/src/articubot_one/worlds/obstacles.world
#간단 nav2 런치파일
ros2 launch articubot_one localization_launch.py
# nav2 with costmap
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
