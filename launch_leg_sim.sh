#!/bin/bash

ROBOT_TYPE="laika"

# 窗口1：启动Gazebo环境
gnome-terminal --title="GAZEBO" -- bash -c \
  "echo '窗口1: Gazebo环境 (ROBOT_TYPE=$ROBOT_TYPE)'; 
   export ROBOT_TYPE=$ROBOT_TYPE; 
   source devel/setup.bash;
   rospack find leg_urdf20250417_1;
   roslaunch legged_unitree_description load_urdf.launch; 
   exec bash"

sleep 3

# 窗口2：加载控制器
gnome-terminal --title="CONTROLLER" -- bash -c \
  "echo '窗口2: 控制器加载 (ROBOT_TYPE=$ROBOT_TYPE)'; 
   export ROBOT_TYPE=$ROBOT_TYPE; 
   source devel/setup.bash;
   rospack find leg_urdf20250417_1;
   roslaunch legged_controllers load_controller.launch cheater:=false; 
   exec bash"

# 窗口3：启动RQT界面
# gnome-terminal --title="RQT" -- bash -c \
#   "echo '窗口3: RQT控制界面（手动点击 Plugins > Robot Tools > Controller Manager）'; 
#    source devel/setup.bash;
#    rqt;
#    exec bash"
