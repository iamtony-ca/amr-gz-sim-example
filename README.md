# how to use
source /opt/ros/jazzy/setup.bash && source /root/work_ws/install/setup.bash

ros2 launch mobile_robot_gz_sim simulation.launch.py headless:=False use_rviz:=True


ros2 launch mobile_robot_gz_sim simulation_static.launch.py headless:=False use_rviz:=True